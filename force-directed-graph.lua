local table = require 'ext.table'
local class = require 'ext.class'
local assert = require 'ext.assert'
local gl = require 'gl'
local GLArrayBuffer = require 'gl.arraybuffer'
local GLElementArrayBuffer = require 'gl.elementarraybuffer'
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
pointsize = 3
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
			for i,n in ipairs(app.nodes) do
				n.pos.x = n.pos.x + n.vel.x * dt
				n.pos.y = n.pos.y + n.vel.y * dt
				n.pos.z = n.pos.z + n.vel.z * dt
				n.vel.x = n.vel.x + n.acc.x * dt
				n.vel.y = n.vel.y + n.acc.y * dt
				n.vel.z = n.vel.z + n.acc.z * dt
			end
		end,
	},
	{
		name = 'RK4',
		update = function(integrator, app)
			for _,n in ipairs(app.nodes) do
				n.pushVel.x, n.pushVel.y, n.pushVel.z = n.vel.x, n.vel.y, n.vel.z
				n.pushPos.x, n.pushPos.y, n.pushPos.z = n.pos.x, n.pos.y, n.pos.z
			end
			app:calcAccel()
			for _,n in ipairs(app.nodes) do
				n.k1a.x, n.k1a.y, n.k1a.z = n.acc.x, n.acc.y, n.acc.z
				n.k1v.x, n.k1v.y, n.k1v.z = n.vel.x, n.vel.y, n.vel.z
				n.vel.x = n.pushVel.x + n.k1a.x * dt * .5
				n.vel.y = n.pushVel.y + n.k1a.y * dt * .5
				n.vel.z = n.pushVel.z + n.k1a.z * dt * .5
				n.pos.x = n.pushPos.x + n.k1v.x * dt * .5
				n.pos.y = n.pushPos.y + n.k1v.y * dt * .5
				n.pos.z = n.pushPos.z + n.k1v.z * dt * .5
			end
			app:calcAccel()
			for _,n in ipairs(app.nodes) do
				n.k2a.x, n.k2a.y, n.k2a.z = n.acc.x, n.acc.y, n.acc.z
				n.k2v.x, n.k2v.y, n.k2v.z = n.vel.x, n.vel.y, n.vel.z
				n.vel.x = n.pushVel.x + n.k2a.x * dt * .5
				n.vel.y = n.pushVel.y + n.k2a.y * dt * .5
				n.vel.z = n.pushVel.z + n.k2a.z * dt * .5
				n.pos.x = n.pushPos.x + n.k2v.x * dt * .5
				n.pos.y = n.pushPos.y + n.k2v.y * dt * .5
				n.pos.z = n.pushPos.z + n.k2v.z * dt * .5
			end
			app:calcAccel()
			for _,n in ipairs(app.nodes) do
				n.k3a.x, n.k3a.y, n.k3a.z = n.acc.x, n.acc.y, n.acc.z
				n.k3v.x, n.k3v.y, n.k3v.z = n.vel.x, n.vel.y, n.vel.z
				n.vel.x = n.pushVel.x + n.k3a.x * dt
				n.vel.y = n.pushVel.y + n.k3a.y * dt
				n.vel.z = n.pushVel.z + n.k3a.z * dt
				n.pos.x = n.pushPos.x + n.k3v.x * dt
				n.pos.y = n.pushPos.y + n.k3v.y * dt
				n.pos.z = n.pushPos.z + n.k3v.z * dt
			end
			app:calcAccel()
			for _,n in ipairs(app.nodes) do
				n.k4a.x, n.k4a.y, n.k4a.z = n.acc.x, n.acc.y, n.acc.z
				n.k4v.x, n.k4v.y, n.k4v.z = n.vel.x, n.vel.y, n.vel.z
				n.vel.x = n.pushVel.x + (n.k1a.x + n.k2a.x * 2 + n.k3a.x * 2 + n.k4a.x) / 6 * dt
				n.vel.y = n.pushVel.y + (n.k1a.y + n.k2a.y * 2 + n.k3a.y * 2 + n.k4a.y) / 6 * dt
				n.vel.z = n.pushVel.z + (n.k1a.z + n.k2a.z * 2 + n.k3a.z * 2 + n.k4a.z) / 6 * dt
				n.pos.x = n.pushPos.x + (n.k1v.x + n.k2v.x * 2 + n.k3v.x * 2 + n.k4v.x) / 6 * dt
				n.pos.y = n.pushPos.y + (n.k1v.y + n.k2v.y * 2 + n.k3v.y * 2 + n.k4v.y) / 6 * dt
				n.pos.z = n.pushPos.z + (n.k1v.z + n.k2v.z * 2 + n.k3v.z * 2 + n.k4v.z) / 6 * dt
			end
		end,
	},
	{
		name = 'P.C.',
		update = function(integrator, app)
			app:calcAccel()
			for _,n in ipairs(app.nodes) do
				n.pushVel.x, n.pushVel.y, n.pushVel.z = n.vel.x, n.vel.y, n.vel.z
				n.pushPos.x, n.pushPos.y, n.pushPos.z = n.pos.x, n.pos.y, n.pos.z
				n.pushAcc.x, n.pushAcc.y, n.pushAcc.z = n.acc.x, n.acc.y, n.acc.z
			end
			for iter=1,30 do
				app:calcAccel()
				for _,n in ipairs(app.nodes) do
					n.pos.x = n.pushPos.x + (n.pushVel.x + n.vel.x) * (dt * .5)
					n.pos.y = n.pushPos.y + (n.pushVel.y + n.vel.y) * (dt * .5)
					n.pos.z = n.pushPos.z + (n.pushVel.z + n.vel.z) * (dt * .5)
					n.vel.x = n.pushVel.x + (n.pushAcc.x + n.acc.x) * (dt * .5)
					n.vel.y = n.pushVel.y + (n.pushAcc.y + n.acc.y) * (dt * .5)
					n.vel.z = n.pushVel.z + (n.pushAcc.z + n.acc.z) * (dt * .5)
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
		local name, pos
		if type(node) == 'table' then
			name = node.name
			pos = node.pos
		else
			name = tostring(node)
			pos = vec3f(crand(), crand(), crand())
		end
		return Node{
			name = tostring(name),
			pos = pos,
			vel = vec3f(0,0,0),
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

	for i=1,#self.nodes-1 do
		local n = self.nodes[i]
		for j=i+1,#self.nodes do
			local n2 = self.nodes[j]
			local pull = pullcoeff
			-- from n to n2
			local dx = n2.pos.x - n.pos.x
			local dy = n2.pos.y - n.pos.y
			local dz = n2.pos.z - n.pos.z
			local dist = math.max(math.sqrt(dx*dx + dy*dy + dz*dz), 1e-4)

			local forceScale = dt * (
				(dist - restdist) / dist * self.weights(i, j)
				- repel / (dist * dist)
			)
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

	-- TODO interleave attrs or nah?
	self.vertexGPU = GLArrayBuffer{
		dim = 3,
		useVec = true,
	}
	do
		local vertexCPU = self.vertexGPU:beginUpdate()
		for _,n in ipairs(self.nodes) do
			vertexCPU:emplace_back():set(0,0,0)
		end
		self.vertexGPU:endUpdate()
	end

	self.colorGPU = GLArrayBuffer{
		dim = 3,
		useVec = true,
		-- TODO allow .count for useVec?
	}
	-- ... instead of .count I do this ...
	do
		local colorCPU = self.colorGPU:beginUpdate()
		for _,n in ipairs(self.nodes) do
			colorCPU:emplace_back():set(1,1,1)
		end
		self.colorGPU:endUpdate()
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
			mode = gl.GL_POINTS,
			count = #self.nodes,
		},
		vertexes = self.vertexGPU,
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
		vertexes = self.vertexGPU,
		attrs = {
			color = {
				buffer = self.colorGPU,
			},
		},
	}
end

local pushDraggingPos = vec3f()
local pushDraggingVel = vec3f()
function App:update()
	if not self.hasFocus then
		sdl.SDL_Delay(300)
		return
	end

	if running then
		local integrator = assert.index(integrators, integratorIndex, "unknown integrator")

		local draggingNode = self.draggingNodeIndex and self.nodes[self.draggingNodeIndex]
		if draggingNode then
			pushDraggingPos:set(draggingNode.pos:unpack())
			pushDraggingVel:set(draggingNode.vel:unpack())
		end

		integrator:update(self)

		-- TODO don't use n.pos , just use vertexCPU
		-- then TODO use transform feedback buffer for integration
		local vertexCPU = self.vertexGPU.vec
		local vertexPtr = vertexCPU.v + 0
		for i,n in ipairs(self.nodes) do
			n.vel.x = n.vel.x * veldecay	-- decay / bound
			n.vel.y = n.vel.y * veldecay
			n.vel.z = n.vel.z * veldecay
			n.pos.x = n.pos.x * posdecay	-- decay / bound
			n.pos.y = n.pos.y * posdecay
			n.pos.z = n.pos.z * posdecay
			vertexPtr.x, vertexPtr.y, vertexPtr.z = n.pos.x, n.pos.y, n.pos.z
			vertexPtr = vertexPtr + 1
		end

		if draggingNode then
			draggingNode.pos:set(pushDraggingPos:unpack())
			draggingNode.vel:set(pushDraggingVel:unpack())
		end

		self.vertexGPU
			:bind()
			:updateData(
				0,
				ffi.sizeof(vec3f) * #self.nodes,
				vertexCPU.v)
	end

	gl.glClear(bit.bor(gl.GL_COLOR_BUFFER_BIT, gl.GL_DEPTH_BUFFER_BIT))

	gl.glPointSize(pointsize)
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
	self.pointObj:draw()

	gl.glEnable(gl.GL_BLEND)
	gl.glBlendFunc(gl.GL_ONE, gl.GL_ONE)

	self.lineObj.uniforms.mvProjMat = self.view.mvProjMat.ptr
	self.lineObj:draw()

	gl.glDisable(gl.GL_BLEND)

	App.super.update(self)
end

function App:reset()
	for _,n in ipairs(self.nodes) do
		n.pos:set(crand(), crand(), crand())
		n.vel:set(0,0,0)
	end
end

function App:updateGUI()
	ig.luatableCheckbox('running', _G, 'running')
	ig.luatableInputFloat('pointsize', _G, 'pointsize')
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

	for i,n in ipairs(self.nodes) do
		local x, y, z, w = self.view.mvProjMat:mul4x4v4(n.pos:unpack())
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
			local dx = self.mouse.deltaPos.x * self.width
			local dy = self.mouse.deltaPos.y * self.height
			local dist = (hoverNode.pos - self.view.pos):dot(-self.view.angle:zAxis())
			local move = self.view.angle:rotate(vec3f(dx,dy,0) * (dist * 2 / self.height))
			hoverNode.pos.x = hoverNode.pos.x + move.x
			hoverNode.pos.y = hoverNode.pos.y + move.y
			hoverNode.pos.z = hoverNode.pos.z + move.z
			-- ... then drag the current mouse-over node
			-- ... and don't update any more
		end
	else
		local oldHoverNodeIndex = self.hoverNodeIndex

		-- not dragging? search for new hoverNode
		self.hoverNodeIndex = nil
		local mouseDistThreshold = 20
		if bestMouseNodeIndex
		and bestMouseDistSq < mouseDistThreshold * mouseDistThreshold
		then
			self.hoverNodeIndex = bestMouseNodeIndex
		end

		-- got a new node? update colors
		if self.hoverNodeIndex ~= oldHoverNodeIndex then
			self.colorGPU:bind()
			local colorCPU = self.colorGPU.vec
			if oldHoverNodeIndex then
				local oldIndex = oldHoverNodeIndex - 1
				colorCPU.v[oldIndex]:set(1,1,1)	-- clear old
				self.colorGPU:updateData(
					ffi.sizeof(vec3f) * oldIndex,
					ffi.sizeof(vec3f),
					colorCPU.v + oldIndex)
			end
			if self.hoverNodeIndex then
				local newIndex = self.hoverNodeIndex - 1
				colorCPU.v[newIndex]:set(1,1,0)
				self.colorGPU:updateData(
					ffi.sizeof(vec3f) * newIndex,
					ffi.sizeof(vec3f),
					colorCPU.v + newIndex)
			end
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
