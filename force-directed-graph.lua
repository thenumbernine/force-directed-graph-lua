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
local vec3d = require 'vec-ffi.vec3d'
local vec4d = require 'vec-ffi.vec4d'

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
			for _,n in ipairs(app.nodes) do
				n.pos = n.pos + n.vel * dt
				n.vel = n.vel + n.acc * dt
			end
		end,
	},
	{
		name = 'RK4',
		update = function(integrator, app)
			for _,n in ipairs(app.nodes) do
				n.pushVel = vec3d(n.vel:unpack())
				n.pushPos = vec3d(n.pos:unpack())
			end
			app:calcAccel()
			for _,n in ipairs(app.nodes) do
				n.k1a = vec3d(n.acc:unpack())
				n.k1v = vec3d(n.vel:unpack())
				n.vel = n.pushVel + n.k1a * dt/2
				n.pos = n.pushPos + n.k1v * dt/2
			end
			app:calcAccel()
			for _,n in ipairs(app.nodes) do
				n.k2a = vec3d(n.acc:unpack())
				n.k2v = vec3d(n.vel:unpack())
				n.vel = n.pushVel + n.k2a * dt/2
				n.pos = n.pushPos + n.k2v * dt/2
			end
			app:calcAccel()
			for _,n in ipairs(app.nodes) do
				n.k3a = vec3d(n.acc:unpack())
				n.k3v = vec3d(n.vel:unpack())
				n.vel = n.pushVel + n.k3a * dt
				n.pos = n.pushPos + n.k3v * dt
			end
			app:calcAccel()
			for _,n in ipairs(app.nodes) do
				n.k4a = vec3d(n.acc:unpack())
				n.k4v = vec3d(n.vel:unpack())
				n.vel = n.pushVel + (n.k1a + n.k2a * 2 + n.k3a * 2 + n.k4a) / 6 * dt
				n.pos = n.pushPos + (n.k3v + n.k2v * 2 + n.k3v * 2 + n.k4v) / 6 * dt
			end
		end,
	},
	{
		name = 'P.C.',
		update = function(integrator, app)
			app:calcAccel()
			for _,n in ipairs(app.nodes) do
				n.pushVel = vec3d(n.vel:unpack())
				n.pushPos = vec3d(n.pos:unpack())
				n.pushAcc = vec3d(n.acc:unpack())
			end
			for iter=1,30 do
				app:calcAccel()
				for _,n in ipairs(app.nodes) do
					n.pos = n.pushPos + (.5 * dt) * (n.pushVel + n.vel)
					n.vel = n.pushVel + (.5 * dt) * (n.pushAcc + n.acc)
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
			pos = vec3d(crand(), crand(), crand())
		end
		return Node{
			name = tostring(name),
			pos = pos,
			screenpos = vec4d(),
			vel = vec3d(0,0,0),
			acc = vec3d(0,0,0),
		}
	end)
	self.weights = args.weights
	return App.super.init(self, args, ...)
end

function App:calcAccel()
	for i,n in ipairs(self.nodes) do
		n.acc = vec3d(0,0,0)
	end

	-- TODO predictor-corrector here, i.e. lazy implicit solver
	for i,n in ipairs(self.nodes) do
		for j,n2 in ipairs(self.nodes) do
			if i ~= j then
				local pull = pullcoeff
				local diff = n2.pos - n.pos	-- from n to n2
				local dist = math.max(diff:length(), 1e-4)
				--local dir = diff / dist

				--local force = dir * (pull * (dist - restdist) - repel / (dist * dist))
				local force = diff * (dist - restdist) / dist * self.weights(i, j)
							- diff * repel / (dist * dist)

				if i ~= self.hoverNodeIndex then
					n.acc = n.acc + force * dt
				end
				if j ~= self.hoverNodeIndex then
					n2.acc = n2.acc - force * dt
				end
			end
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

function App:update()
	if not self.hasFocus then
		sdl.SDL_Delay(300)
		return
	end

	if running then
		local integrator = assert.index(integrators, integratorIndex, "unknown integrator")
		integrator:update(self)
		for _,n in ipairs(self.nodes) do
			n.vel = n.vel * veldecay	-- decay / bound
			n.pos = n.pos * posdecay	-- decay / bound
		end
	
		local vertexGPU = self.vertexGPU
		local vertexCPU = vertexGPU:beginUpdate()
		for _,n in ipairs(self.nodes) do
			local v = vertexCPU:emplace_back()
			v.x, v.y, v.z = n.pos:unpack()
		end
		vertexGPU:endUpdate()
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
		n.pos = vec3d( crand(), crand(), crand() )
		n.vel = vec3d(0,0,0)
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
		n.screenpos:set(x,y,z,w)
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

	-- if we are over a node then try to drag it
	self.hoverNodeIsDragging = false
	if self.hoverNodeIndex
	and self.mouse.leftDown
	then
		local hoverNode = assert.index(self.nodes, self.hoverNodeIndex)
		self.hoverNodeIsDragging = true
		if self.mouse.deltaPos.x ~= 0
		or self.mouse.deltaPos.y ~= 0
		then
			local dx = self.mouse.deltaPos.x * self.width
			local dy = self.mouse.deltaPos.y * self.height
			local dist = (hoverNode.pos - self.view.pos):dot(-self.view.angle:zAxis())
			hoverNode.pos = hoverNode.pos + self.view.angle:rotate(vec3d(dx,dy,0) * (dist * 2 / self.height))
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
	if self.hoverNodeIsDragging then return end
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
