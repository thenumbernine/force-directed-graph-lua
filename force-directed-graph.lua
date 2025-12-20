local table = require 'ext.table'
local class = require 'ext.class'
local assert = require 'ext.assert'
local gl = require 'gl'
local GLArrayBuffer = require 'gl.arraybuffer'
local GLElementArrayBuffer = require 'gl.elementarraybuffer'
local GLProgram = require 'gl.program'
local GLVertexArray = require 'gl.vertexarray'
local GLSceneObject = require 'gl.sceneobject'
local sdl = require 'sdl'
local ig = require 'imgui'
local ffi = require 'ffi'
local vec3f = require 'vec-ffi.vec3f'

local App = require 'imgui.appwithorbit'()
App.title = 'force directed graph'

App.viewDist = 2

local function crand() return math.random() * 2 - 1 end

local Node = class()
function Node:init(args)
	for k,v in pairs(args) do
		self[k] = v
	end
end

running = true
pointSize = 3
dt = .1
pullcoeff = 1
drawcoeff = 1
veldecay = .99
posdecay = .999
repel = 1
restdist = .3

local integrators = table{
	{
		name = 'F.E.',
		update = function(integrator, app)
			app:calcAccel()
			local npos = app.posGPU.vec.v + 0
			local nvel = app.velGPU.vec.v + 0
			for i,n in ipairs(app.nodes) do
				npos.x = npos.x + nvel.x * dt
				npos.y = npos.y + nvel.y * dt
				npos.z = npos.z + nvel.z * dt
				nvel.x = nvel.x + n.acc.x * dt
				nvel.y = nvel.y + n.acc.y * dt
				nvel.z = nvel.z + n.acc.z * dt
				npos = npos + 1
				nvel = nvel + 1
			end
		end,
	},
	{
		name = 'RK4',
		update = function(integrator, app)
			local npos = app.posGPU.vec.v + 0
			local nvel = app.velGPU.vec.v + 0
			for _,n in ipairs(app.nodes) do
				n.pushVel.x, n.pushVel.y, n.pushVel.z = nvel.x, nvel.y, nvel.z
				n.pushPos.x, n.pushPos.y, n.pushPos.z = npos.x, npos.y, npos.z
				npos = npos + 1
				nvel = nvel + 1
			end

			app:calcAccel()
			local npos = app.posGPU.vec.v + 0
			local nvel = app.velGPU.vec.v + 0
			for _,n in ipairs(app.nodes) do
				n.k1a.x, n.k1a.y, n.k1a.z = n.acc.x, n.acc.y, n.acc.z
				n.k1v.x, n.k1v.y, n.k1v.z = nvel.x, nvel.y, nvel.z
				nvel.x = n.pushVel.x + n.k1a.x * dt * .5
				nvel.y = n.pushVel.y + n.k1a.y * dt * .5
				nvel.z = n.pushVel.z + n.k1a.z * dt * .5
				npos.x = n.pushPos.x + n.k1v.x * dt * .5
				npos.y = n.pushPos.y + n.k1v.y * dt * .5
				npos.z = n.pushPos.z + n.k1v.z * dt * .5
				npos = npos + 1
				nvel = nvel + 1
			end

			app:calcAccel()
			local npos = app.posGPU.vec.v + 0
			local nvel = app.velGPU.vec.v + 0
			for _,n in ipairs(app.nodes) do
				n.k2a.x, n.k2a.y, n.k2a.z = n.acc.x, n.acc.y, n.acc.z
				n.k2v.x, n.k2v.y, n.k2v.z = nvel.x, nvel.y, nvel.z
				nvel.x = n.pushVel.x + n.k2a.x * dt * .5
				nvel.y = n.pushVel.y + n.k2a.y * dt * .5
				nvel.z = n.pushVel.z + n.k2a.z * dt * .5
				npos.x = n.pushPos.x + n.k2v.x * dt * .5
				npos.y = n.pushPos.y + n.k2v.y * dt * .5
				npos.z = n.pushPos.z + n.k2v.z * dt * .5
				npos = npos + 1
				nvel = nvel + 1
			end

			app:calcAccel()
			local npos = app.posGPU.vec.v + 0
			local nvel = app.velGPU.vec.v + 0
			for _,n in ipairs(app.nodes) do
				n.k3a.x, n.k3a.y, n.k3a.z = n.acc.x, n.acc.y, n.acc.z
				n.k3v.x, n.k3v.y, n.k3v.z = nvel.x, nvel.y, nvel.z
				nvel.x = n.pushVel.x + n.k3a.x * dt
				nvel.y = n.pushVel.y + n.k3a.y * dt
				nvel.z = n.pushVel.z + n.k3a.z * dt
				npos.x = n.pushPos.x + n.k3v.x * dt
				npos.y = n.pushPos.y + n.k3v.y * dt
				npos.z = n.pushPos.z + n.k3v.z * dt
				npos = npos + 1
				nvel = nvel + 1
			end

			app:calcAccel()
			local npos = app.posGPU.vec.v + 0
			local nvel = app.velGPU.vec.v + 0
			for _,n in ipairs(app.nodes) do
				n.k4a.x, n.k4a.y, n.k4a.z = n.acc.x, n.acc.y, n.acc.z
				n.k4v.x, n.k4v.y, n.k4v.z = nvel.x, nvel.y, nvel.z
				nvel.x = n.pushVel.x + (n.k1a.x + n.k2a.x * 2 + n.k3a.x * 2 + n.k4a.x) / 6 * dt
				nvel.y = n.pushVel.y + (n.k1a.y + n.k2a.y * 2 + n.k3a.y * 2 + n.k4a.y) / 6 * dt
				nvel.z = n.pushVel.z + (n.k1a.z + n.k2a.z * 2 + n.k3a.z * 2 + n.k4a.z) / 6 * dt
				npos.x = n.pushPos.x + (n.k1v.x + n.k2v.x * 2 + n.k3v.x * 2 + n.k4v.x) / 6 * dt
				npos.y = n.pushPos.y + (n.k1v.y + n.k2v.y * 2 + n.k3v.y * 2 + n.k4v.y) / 6 * dt
				npos.z = n.pushPos.z + (n.k1v.z + n.k2v.z * 2 + n.k3v.z * 2 + n.k4v.z) / 6 * dt
				npos = npos + 1
				nvel = nvel + 1
			end
		end,
	},
	{
		name = 'P.C.',
		update = function(integrator, app)
			app:calcAccel()
			local npos = app.posGPU.vec.v + 0
			local nvel = app.velGPU.vec.v + 0
			for _,n in ipairs(app.nodes) do
				n.pushVel.x, n.pushVel.y, n.pushVel.z = nvel.x, nvel.y, nvel.z
				n.pushPos.x, n.pushPos.y, n.pushPos.z = npos.x, npos.y, npos.z
				n.pushAcc.x, n.pushAcc.y, n.pushAcc.z = n.acc.x, n.acc.y, n.acc.z
				npos = npos + 1
				nvel = nvel + 1
			end
			for iter=1,30 do
				app:calcAccel()
				local npos = app.posGPU.vec.v + 0
				local nvel = app.velGPU.vec.v + 0
				for _,n in ipairs(app.nodes) do
					npos.x = n.pushPos.x + (n.pushVel.x + nvel.x) * (dt * .5)
					npos.y = n.pushPos.y + (n.pushVel.y + nvel.y) * (dt * .5)
					npos.z = n.pushPos.z + (n.pushVel.z + nvel.z) * (dt * .5)
					nvel.x = n.pushVel.x + (n.pushAcc.x + n.acc.x) * (dt * .5)
					nvel.y = n.pushVel.y + (n.pushAcc.y + n.acc.y) * (dt * .5)
					nvel.z = n.pushVel.z + (n.pushAcc.z + n.acc.z) * (dt * .5)
					npos = npos + 1
					nvel = nvel + 1
				end
			end
		end,
	},
}
local integratorNames = integrators:mapi(function(integrator) return integrator.name end)
local integratorIndeForName = integrators:mapi(function(integrator, index) return index, integrator.name end):setmetatable(nil)
integratorIndex = assert.index(integratorIndeForName, 'RK4')



--[[
config format:
nodes = {
	'node name 1',
	'node name 2',
	...
}

weights = function(nodeIndex1, nodeIndex2) return strength
--]]
function App:init(args, ...)
	self.nodes = table.mapi(args.nodes, function(node)
		if type(node) == 'string' then
			node = {name = node}
		end
		return Node{
			name = tostring(node.name),

			-- only used for init, then copied into posGPU.vec:
			pos = node.pos or vec3f(crand(), crand(), crand()),

			color = node.color or vec3f(1,1,1),

			acc = vec3f(0,0,0),

			-- integrator helpers
			pushPos = vec3f(),
			pushVel = vec3f(),
			pushAcc = vec3f(),
			k1a = vec3f(),
			k1v = vec3f(),
			k2a = vec3f(),
			k2v = vec3f(),
			k3a = vec3f(),
			k3v = vec3f(),
			k4a = vec3f(),
			k4v = vec3f(),
		}
	end)
	self.weights = args.weights
	return App.super.init(self, args, ...)
end

function App:calcAccel()
	for i,n in ipairs(self.nodes) do
		n.acc.x, n.acc.y, n.acc.z = 0, 0, 0
	end

	local posCPU = self.posGPU.vec
	for i=1,#self.nodes-1 do
		local n = self.nodes[i]
		local npos = posCPU.v + (i-1)
		for j=i+1,#self.nodes do
			local n2 = self.nodes[j]
			local n2pos = posCPU.v + (j-1)
			local pull = pullcoeff
			-- from n to n2
			local dx = n2pos.x - npos.x
			local dy = n2pos.y - npos.y
			local dz = n2pos.z - npos.z
			local dist = math.max(math.sqrt(dx*dx + dy*dy + dz*dz), 1e-4)

			local forceScale = -repel / (dist * dist)
			local w = self.weights(i,j)
			-- what should 'w' influence?
			-- rest-distance?
			-- force?
			-- both?
			if w ~= 0 then
				forceScale = forceScale + (1 - restdist / dist) * w
			end
			forceScale = forceScale * dt
			local fx = dx * forceScale
			local fy = dy * forceScale
			local fz = dz * forceScale

			n.acc.x = n.acc.x + fx
			n.acc.y = n.acc.y + fy
			n.acc.z = n.acc.z + fz
			n2.acc.x = n2.acc.x - fx
			n2.acc.y = n2.acc.y - fy
			n2.acc.z = n2.acc.z - fz
		end
	end
end

function App:initGL(...)
	App.super.initGL(self, ...)

	self.posGPU = GLArrayBuffer{
		dim = 3,
		useVec = true,
		count = #self.nodes,
		usage = gl.GL_DYNAMIC_DRAW,
	}
	do
		assert.len(self.posGPU.vec, #self.nodes)
		local npos = self.posGPU.vec.v + 0
		for _,n in ipairs(self.nodes) do
			npos:set(n.pos:unpack())
			n.pos = nil
			npos = npos + 1
		end
		self.posGPU
			:bind()
			:updateData()
	end

	self.velGPU = GLArrayBuffer{
		dim = 3,
		useVec = true,
		count = #self.nodes,
	}
	do
		assert.len(self.velGPU.vec, #self.nodes)
		local nvel = self.velGPU.vec.v + 0
		for _,n in ipairs(self.nodes) do
			nvel:set(0,0,0)
			nvel = nvel + 1
		end
		self.velGPU
			:bind()
			:updateData()
	end

	self.colorGPU = GLArrayBuffer{
		dim = 3,
		useVec = true,
		count = #self.nodes
	}
	do
		assert.len(self.colorGPU.vec, #self.nodes)
		local ncolor = self.colorGPU.vec.v + 0
		for _,n in ipairs(self.nodes) do
			ncolor:set(n.color:unpack())
			n.color = nil
			ncolor = ncolor + 1
		end
		self.colorGPU
			:bind()
			:updateData()
	end

	self.pointObj = GLSceneObject{
		program = {
			version = 'latest',
			precision = 'best',
			vertexCode = [[
layout(location=0) in vec3 vertex;
layout(location=1) in vec3 color;
out vec3 colorv;
uniform mat4 mvProjMat;
uniform float pointSize;
uniform int hoverNodeIndex;
void main() {
	colorv = color;
	gl_Position = mvProjMat * vec4(vertex, 1.);
	gl_PointSize = pointSize;
	if (hoverNodeIndex == gl_VertexID) gl_PointSize += 2.;
}
]],
			fragmentCode = [[
in vec3 colorv;
out vec4 fragColor;
void main() {
	fragColor = vec4(colorv, 1.);
}
]],
		},
		geometry = {
			mode = gl.GL_POINTS,
			count = #self.nodes,
		},
		vertexes = self.posGPU,
		attrs = {
			color = {
				buffer = self.colorGPU,
			},
		},
	}

	-- assume our weights don't change during runtime
	-- also assume weight(i,j) function is symmetric
	local lineIndexes = table()
	for i=1,#self.nodes-1 do
		for j=i+1,#self.nodes do
			local w = self.weights(i,j)
			if w ~= 0 then
				lineIndexes:insert(i-1)
				lineIndexes:insert(j-1)
			end
		end
	end
	local lineIndexesGPU = GLElementArrayBuffer{
		data = lineIndexes,
	}
	self.lineObj = GLSceneObject{
		program = {
			version = 'latest',
			precision = 'best',
			vertexCode = [[
layout(location=0) in vec3 vertex;
layout(location=1) in vec3 color;
out vec3 colorv;
uniform mat4 mvProjMat;
void main() {
	colorv = color;
	gl_Position = mvProjMat * vec4(vertex, 1.);
}
]],
			fragmentCode = [[
in vec3 colorv;
out vec4 fragColor;
void main() {
	fragColor = vec4(colorv, 1.);
}
]],
		},
		geometry = {
			mode = gl.GL_LINES,
			indexes = lineIndexesGPU,
		},
		vertexes = self.posGPU,
		attrs = {
			color = {
				buffer = self.colorGPU,
			},
		},
	}

--[==[ transform-feedback pingpong
	self.accGPU = GLArrayBuffer{
		dim = 3,
		useVec = true,
		count = #self.nodes,
	}
	self.newPosGPU = GLArrayBuffer{
		dim = 3,
		useVec = true,
		count = #self.nodes,
	}
	self.newVelGPU = GLArrayBuffer{
		dim = 3,
		useVec = true,
		count = #self.nodes,
	}
	self.calcForceProgram = GLProgram{
		version = 'latest',
		precision = 'best',
		vertexCode = [[
layout(location=0) in vec3 pos;
layout(location=1) in vec3 vel;
layout(location=0) out vec3 acc;
void main() {
	acc = vec3(0., 0., 0.);

#error TODO how to encode the graph sparse matrix
}
]],
	}:unbind()

	self.integrateEulerProgram = GLProgram{
		version = 'latest',
		precision = 'best',
		vertexCode = [[
location(layout=0) in vec3 pos;
location(layout=1) in vec3 vel;
location(layout=2) in vec3 acc;
location(layout=0) out vec3 newPos;
location(layout=1) out vec3 newVel;
uniform float dt;
void main() {
	newPos = pos + vel * dt;
	newVel = vel + acc * dt;
}
]],
		transformFeedback = {
			'newPos',
			'newVel',
			mode = 'separate',
		},
	}:unbind()

	-- TODO this when you run it
	--self.integrateEulerProgram.uniforms.dt = dt

	self.integrateEulerVAO = GLVertexArray{
		program = self.integrateEulerProgram,
		attrs = {
			pos = {
				buffer = self.posGPU,
			},
			vel = {
				buffer = self.velGPU,
			},
			acc = {
				buffer = self.accGPU,
			},
		},
	}
--]==]
end

local pushDraggingPos = vec3f()
local pushDraggingVel = vec3f()
function App:update()
	local posCPU = self.posGPU.vec
	local velCPU = self.velGPU.vec

	if not self.hasFocus then
		sdl.SDL_Delay(300)
		return
	end

	if running then
		local integrator = assert.index(integrators, integratorIndex, "unknown integrator")

		local draggingNode = self.draggingNodeIndex and self.nodes[self.draggingNodeIndex]
		local draggingNodePos = draggingNode and (posCPU.v + (self.draggingNodeIndex-1))
		local draggingNodeVel = draggingNode and (velCPU.v + (self.draggingNodeIndex-1))
		if draggingNode then
			pushDraggingPos:set(draggingNodePos:unpack())
			pushDraggingVel:set(draggingNodeVel:unpack())
		end

		integrator:update(self)

		-- TODO use transform feedback buffer for integration
		local npos = posCPU.v + 0
		local nvel = velCPU.v + 0
		for i,n in ipairs(self.nodes) do
			nvel.x = nvel.x * veldecay	-- decay / bound
			nvel.y = nvel.y * veldecay
			nvel.z = nvel.z * veldecay
			npos.x = npos.x * posdecay	-- decay / bound
			npos.y = npos.y * posdecay
			npos.z = npos.z * posdecay
			npos = npos + 1
			nvel = nvel + 1
		end

		if draggingNode then
			draggingNodePos:set(pushDraggingPos:unpack())
			draggingNodeVel:set(pushDraggingVel:unpack())
		end

		self.posGPU
			:bind()
			:updateData()
	end

	gl.glClear(bit.bor(gl.GL_COLOR_BUFFER_BIT, gl.GL_DEPTH_BUFFER_BIT))

	gl.glHint(gl.GL_POINT_SMOOTH, gl.GL_NICEST)
	gl.glHint(gl.GL_LINE_SMOOTH, gl.GL_NICEST)
	gl.glHint(gl.GL_POLYGON_SMOOTH, gl.GL_NICEST)
	gl.glEnable(gl.GL_POINT_SMOOTH)
	gl.glEnable(gl.GL_LINE_SMOOTH)
	gl.glEnable(gl.GL_POLYGON_SMOOTH)
	gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
	gl.glEnable(gl.GL_BLEND)

	-- TODO replace this with billboards for nodes
	self.pointObj.uniforms.mvProjMat = self.view.mvProjMat.ptr
	self.pointObj.uniforms.pointSize = pointSize
	self.pointObj.uniforms.hoverNodeIndex = (self.hoverNodeIndex or 0) - 1
	self.pointObj:draw()

	gl.glEnable(gl.GL_BLEND)
	gl.glBlendFunc(gl.GL_ONE, gl.GL_ONE)

	self.lineObj.uniforms.mvProjMat = self.view.mvProjMat.ptr
	self.lineObj:draw()

	gl.glDisable(gl.GL_BLEND)

	App.super.update(self)
end

function App:reset()
	local npos = self.posGPU.vec.v + 0
	local nvel = self.velGPU.vec.v + 0
	for _,n in ipairs(self.nodes) do
		npos:set(crand(), crand(), crand())
		nvel:set(0,0,0)
		npos = npos + 1
		nvel = nvel + 1
	end
	self.posGPU
		:bind()
		:updateData()
end

function App:updateGUI()
	local posCPU = self.posGPU.vec

	ig.luatableCheckbox('running', _G, 'running')
	ig.luatableInputFloat('pointSize', _G, 'pointSize')
	ig.luatableInputFloat('dt', _G, 'dt')
	ig.luatableInputFloat('pullcoeff', _G, 'pullcoeff')
	ig.luatableInputFloat('drawcoeff', _G, 'drawcoeff')
	ig.luatableInputFloat('veldecay', _G, 'veldecay')
	ig.luatableInputFloat('posdecay', _G, 'posdecay')
	ig.luatableInputFloat('repel', _G, 'repel')
	ig.luatableInputFloat('restdist', _G, 'restdist')
	ig.luatableCombo('integrator', _G, 'integratorIndex', integratorNames)
	if ig.igButton'reset' then
		self:reset()
	end

	local mousePos = ig.igGetMousePos()
	local bestMouseDistSq = math.huge
	local bestMouseNodeIndex

	local npos = self.posGPU.vec.v + 0
	for i,n in ipairs(self.nodes) do
		local x, y, z, w = self.view.mvProjMat:mul4x4v4(npos:unpack())
		if x >= -w and x <= w
		and y >= -w and y <= w
		and z >= -w and z <= w
		then
			x = math.floor((.5 + .5 * x / w) * self.width)
			y = math.floor((.5 - .5 * y / w) * self.height)
			z = 1 - z / w
			local dx = x - mousePos.x
			local dy = y - mousePos.y
			local distSq = dx*dx + dy*dy
			if distSq < bestMouseDistSq then
				bestMouseDistSq = distSq
				bestMouseNodeIndex = i
				--print('z', z)
			end

			ig.igPushID_Str(n.name)
			ig.igSetNextWindowPos(
				ig.ImVec2(x,y),
				0,
				ig.ImVec2()
			)
			ig.igBegin(
				n.name,
				nil,
				bit.bor(
					ig.ImGuiWindowFlags_NoDecoration,
					ig.ImGuiWindowFlags_Tooltip
				)
			)

			if i == self.hoverNodeIndex then
				ig.igPushStyleColor_U32(ig.ImGuiCol_Text, 0xff00ffff)
			end
			ig.igText(n.name)
			if i == self.hoverNodeIndex then
				ig.igPopStyleColor(1)
			end

			ig.igEnd()
			ig.igPopID()
		end
		npos = npos + 1
	end

	-- if we are over a node then try to drag it
	self.draggingNodeIndex = nil
	if self.hoverNodeIndex
	and self.mouse.leftDown
	then
		local hoverNode = assert.index(self.nodes, self.hoverNodeIndex)
		self.draggingNodeIndex = self.hoverNodeIndex
		if self.mouse.deltaPos.x ~= 0
		or self.mouse.deltaPos.y ~= 0
		then
			local hoverNodePos = posCPU.v + (self.hoverNodeIndex-1)
			local dx = self.mouse.deltaPos.x * self.width
			local dy = self.mouse.deltaPos.y * self.height
			local dist = (hoverNodePos - self.view.pos):dot(-self.view.angle:zAxis())
			local move = self.view.angle:rotate(vec3f(dx,dy,0) * (dist * 2 / self.height))
			hoverNodePos.x = hoverNodePos.x + move.x
			hoverNodePos.y = hoverNodePos.y + move.y
			hoverNodePos.z = hoverNodePos.z + move.z
			local index0based = self.hoverNodeIndex-1
			local v = self.posGPU.vec.v + index0based
			v.x, v.y, v.z = hoverNodePos:unpack()
			self.posGPU
				:bind()
				:updateData(
					ffi.sizeof(vec3f) * index0based,
					ffi.sizeof(vec3f),
					posCPU.v + index0based)
			-- ... then drag the current mouse-over node
			-- ... and don't update any more
		end
	else
		-- not dragging? search for new hoverNode
		self.hoverNodeIndex = nil
		local mouseDistThreshold = 20
		if bestMouseNodeIndex
		and bestMouseDistSq < mouseDistThreshold * mouseDistThreshold
		then
			self.hoverNodeIndex = bestMouseNodeIndex
		end
	end
end

function App:mouseDownEvent(...)
	if self.draggingNodeIndex then return end
	return App.super.mouseDownEvent(self, ...)
end

function App:event(event)
	if event.type == sdl.SDL_EVENT_WINDOW_FOCUS_GAINED then
		self.hasFocus = true
		return
	elseif event.type == sdl.SDL_EVENT_WINDOW_FOCUS_LOST then
		self.hasFocus = false
		return
	end

	local canHandleMouse = not ig.igGetIO()[0].WantCaptureMouse
	local canHandleKeyboard = not ig.igGetIO()[0].WantCaptureKeyboard

	App.super.event(self, event)

	if canHandleKeyboard then
		if event.type == sdl.SDL_EVENT_KEY_DOWN then
			if event.key.key == sdl.SDLK_SPACE then
				running = not running
			elseif event.key.key == sdl.SDLK_R then
				self:reset()
			end
		end
	end
end

return App
