local table = require 'ext.table'
local class = require 'ext.class'
local gl = require 'gl'
local sdl = require 'sdl'
local ig = require 'imgui'
local ffi = require 'ffi'
local vec3d = require 'vec-ffi.vec3d'
local matrix = require 'matrix'

local App = require 'imgui.appwithorbit'()
App.viewUseGLMatrixMode = true 
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
veldecay = .9
posdecay = .99
repel = .1
restdist = 1

local hoverNode

--[[
config format:
nodes = {
	'node name 1',
	'node name 2',
	...
}

weights = {
	[nodeIndex1] = {[nodeIndex2] = strength, ...},
	...
}
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
			name = name,
			pos = pos,
			vel = vec3d(0,0,0),
			acc = vec3d(0,0,0),
		}
	end)
	self.weights = table(args.weights)
	return App.super.init(self, args, ...)
end

function App:calcAccel()
	for i,n in ipairs(self.nodes) do
		n.acc = vec3d(0,0,0)
	end
	for i,n in ipairs(self.nodes) do
		for j,n2 in ipairs(self.nodes) do
			if i ~= j then
				local pull = pullcoeff
				local diff = n2.pos - n.pos	-- from n to n2
				local dist = math.max(diff:length(), 1e-4)
				--local dir = diff / dist
				
				local restlen = restdist * self.weights[i][j]

				--local force = dir * (pull * (dist - restlen) - repel / (dist * dist))
				local force = diff * (dist - restlen ) / dist 
							- diff * repel / (dist * dist)
				
				n.acc = n.acc + force * dt
				n2.acc = n2.acc - force * dt
			end
		end
	end
end

function App:update()
	if running then
-- [[ Runge-Kutta 4
		for _,n in ipairs(self.nodes) do
			n.pushVel = vec3d(n.vel:unpack())
			n.pushPos = vec3d(n.pos:unpack())
		end
		self:calcAccel()
		for _,n in ipairs(self.nodes) do
			n.k1a = vec3d(n.acc:unpack())
			n.k1v = vec3d(n.vel:unpack())
			n.vel = n.pushVel + n.k1a * dt/2
			n.pos = n.pushPos + n.k1v * dt/2
		end
		self:calcAccel()
		for _,n in ipairs(self.nodes) do
			n.k2a = vec3d(n.acc:unpack())
			n.k2v = vec3d(n.vel:unpack())
			n.vel = n.pushVel + n.k2a * dt/2
			n.pos = n.pushPos + n.k2v * dt/2
		end
		self:calcAccel()
		for _,n in ipairs(self.nodes) do
			n.k3a = vec3d(n.acc:unpack())
			n.k3v = vec3d(n.vel:unpack())
			n.vel = n.pushVel + n.k3a * dt
			n.pos = n.pushPos + n.k3v * dt
		end
		self:calcAccel()
		for _,n in ipairs(self.nodes) do
			n.k4a = vec3d(n.acc:unpack())
			n.k4v = vec3d(n.vel:unpack())
			n.vel = n.pushVel + (n.k1a + n.k2a * 2 + n.k3a * 2 + n.k4a) / 6 * dt
			n.pos = n.pushPos + (n.k3v + n.k2v * 2 + n.k3v * 2 + n.k4v) / 6 * dt
		end
--]]
--[[ forward-Euler
		for _,n in ipairs(self.nodes) do
			n.pos = n.pos + n.vel * dt
			n.vel = n.vel + n.acc * dt
		end
--]]
		
		for _,n in ipairs(self.nodes) do
			n.vel = n.vel * veldecay	-- decay / bound
			n.pos = n.pos * posdecay	-- decay / bound
		end
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
	
	gl.glColor3f(1,1,1)
	gl.glBegin(gl.GL_POINTS)
	for _,n in ipairs(self.nodes) do
		if n == hoverNode then
			gl.glColor3f(1,0,1)
		else
			gl.glColor3f(1,1,1)
		end
		gl.glVertex3dv(n.pos.s)
	end
	gl.glEnd()

	gl.glEnable(gl.GL_BLEND)
	gl.glBlendFunc(gl.GL_ONE, gl.GL_ONE)
	gl.glBegin(gl.GL_LINES)
	for i,n in ipairs(self.nodes) do
		for j,n2 in ipairs(self.nodes) do
			if i ~= j then
				local l = drawcoeff * self.weights[i][j]
				gl.glColor3f(l,l,l)
				gl.glVertex3dv(n.pos.s)
				gl.glVertex3dv(n2.pos.s)
			end
		end
	end
	gl.glEnd()

	App.super.update(self)
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
	if ig.igButton'reset' then
		for _,n in ipairs(self.nodes) do
			n.pos = vec3d( crand(), crand(), crand() )
			n.vel = vec3d(0,0,0)
		end
	end


	local mousePos = ig.igGetMousePos()
	
	-- store screen-space coordinates associated with each point
	local mvmat = ffi.new'double[16]'
	gl.glGetDoublev(gl.GL_MODELVIEW_MATRIX, mvmat)
	mvmat = matrix{4,4}:lambda(function(i,j)
		return tonumber(mvmat[i-1+4*(j-1)])
	end)
	local projmat = ffi.new'double[16]'
	gl.glGetDoublev(gl.GL_PROJECTION_MATRIX, projmat)
	projmat = matrix{4,4}:lambda(function(i,j)
		return tonumber(projmat[i-1+4*(j-1)])
	end)
	local mvpmat = projmat * mvmat
	
	local bestMouseDist = math.huge
	local bestMouseNode 
		
	for i,n in ipairs(self.nodes) do
		n.screenpos = mvpmat * matrix{n.pos.x, n.pos.y, n.pos.z, 1}
		local x, y, z, w = n.screenpos:unpack()
		x = math.floor((.5 + .5 * x / w) * self.width)
		y = math.floor((.5 - .5 * y / w) * self.height)
		z = 1 - z / w
		local dx = x - mousePos.x
		local dy = y - mousePos.y
		local dist = math.sqrt(dx*dx + dy*dy)
		if dist < bestMouseDist then
			bestMouseDist = dist
			bestMouseNode = n
			--print('z', z)
		end
	end
	hoverNode = nil
	local mouseDistThreshold = 20
	if bestMouseNode and bestMouseDist < mouseDistThreshold then
		hoverNode = bestMouseNode
		ig.igBeginTooltip()
		ig.igText(hoverNode.name)
		ig.igEndTooltip()
	end
end

function App:event(event)
	App.super.event(self, event)
	local canHandleMouse = not ig.igGetIO()[0].WantCaptureMouse
	local canHandleKeyboard = not ig.igGetIO()[0].WantCaptureKeyboard
	
	if canHandleKeyboard then
		if event[0].type == sdl.SDL_EVENT_KEYD_OWN then
			if event[0].key.key == sdl.SDLK_SPACE then
				running = not running
			end
		end
	end
end

return App
