local table = require 'ext.table'
local class = require 'ext.class'
local gl = require 'ffi.OpenGL'
local sdl = require 'ffi.sdl'
local ig = require 'ffi.imgui'
local ffi = require 'ffi'
local View = require 'glapp.view'
local Orbit = require 'glapp.orbit'
local ImGuiApp = require 'imguiapp'
local vec3d = require 'vec-ffi.vec3d'
local matrix = require 'matrix'

local App = class(Orbit(View.apply(class(ImGuiApp))))

App.title = 'force directed graph'

App.viewDist = 2

local function crand() return math.random() * 2 - 1 end

local Node = class()
function Node:init(args)
	for k,v in pairs(args) do
		self[k] = v
	end
end

local running = ffi.new('bool[1]', 1)
local pointsize = ffi.new('float[1]', 3)
local dt = ffi.new('float[1]', .01)
local pullcoeff = ffi.new('float[1]', 1)
local drawcoeff = ffi.new('float[1]', 1)
local veldecay = ffi.new('float[1]', .9)
local posdecay = ffi.new('float[1]', .99)
local repel = ffi.new('float[1]', .01)
local restdist = ffi.new('float[1]', .1)

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
				local pull = pullcoeff[0]
				local diff = n2.pos - n.pos	-- from n to n2
				local dist = math.max(diff:length(), 1e-4)
				--local dir = diff / dist
				
				local restlen = restdist[0] * self.weights[i][j]

				--local force = dir * (pull * (dist - restlen) - repel[0] / (dist * dist))
				local force = diff * (dist - restlen ) / dist 
							- diff * repel[0] / (dist * dist)
				
				n.acc = n.acc + force * dt[0]
				n2.acc = n2.acc - force * dt[0]
			end
		end
	end
end

function App:update()
	if running[0] then
-- [[ Runge-Kutta 4
		for _,n in ipairs(self.nodes) do
			n.pushVel = vec3d(n.vel:unpack())
			n.pushPos = vec3d(n.pos:unpack())
		end
		self:calcAccel()
		for _,n in ipairs(self.nodes) do
			n.k1a = vec3d(n.acc:unpack())
			n.k1v = vec3d(n.vel:unpack())
			n.vel = n.pushVel + n.k1a * dt[0]/2
			n.pos = n.pushPos + n.k1v * dt[0]/2
		end
		self:calcAccel()
		for _,n in ipairs(self.nodes) do
			n.k2a = vec3d(n.acc:unpack())
			n.k2v = vec3d(n.vel:unpack())
			n.vel = n.pushVel + n.k2a * dt[0]/2
			n.pos = n.pushPos + n.k2v * dt[0]/2
		end
		self:calcAccel()
		for _,n in ipairs(self.nodes) do
			n.k3a = vec3d(n.acc:unpack())
			n.k3v = vec3d(n.vel:unpack())
			n.vel = n.pushVel + n.k3a * dt[0]
			n.pos = n.pushPos + n.k3v * dt[0]
		end
		self:calcAccel()
		for _,n in ipairs(self.nodes) do
			n.k4a = vec3d(n.acc:unpack())
			n.k4v = vec3d(n.vel:unpack())
			n.vel = n.pushVel + (n.k1a + n.k2a * 2 + n.k3a * 2 + n.k4a) / 6 * dt[0]
			n.pos = n.pushPos + (n.k3v + n.k2v * 2 + n.k3v * 2 + n.k4v) / 6 * dt[0]
		end
--]]
--[[ forward-Euler
		for _,n in ipairs(self.nodes) do
			n.pos = n.pos + n.vel * dt[0]
			n.vel = n.vel + n.acc * dt[0]
		
			n.vel = n.vel * veldecay[0]	-- decay / bound
			n.pos = n.pos * posdecay[0]	-- decay / bound
		end
--]]
	end
	
	gl.glClear(bit.bor(gl.GL_COLOR_BUFFER_BIT, gl.GL_DEPTH_BUFFER_BIT))

	gl.glPointSize(pointsize[0])
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
		gl.glVertex3dv(n.pos:ptr())
	end
	gl.glEnd()

	gl.glEnable(gl.GL_BLEND)
	gl.glBlendFunc(gl.GL_ONE, gl.GL_ONE)
	gl.glBegin(gl.GL_LINES)
	for i,n in ipairs(self.nodes) do
		for j,n2 in ipairs(self.nodes) do
			if i ~= j then
				local l = drawcoeff[0] * self.weights[i][j]
				gl.glColor3f(l,l,l)
				gl.glVertex3dv(n.pos:ptr())
				gl.glVertex3dv(n2.pos:ptr())
			end
		end
	end
	gl.glEnd()

	App.super.update(self)
end

function App:updateGUI()
	ig.igCheckbox('running', running)
	ig.igInputFloat('pointsize', pointsize)
	ig.igInputFloat('dt', dt)
	ig.igInputFloat('pullcoeff', pullcoeff)
	ig.igInputFloat('drawcoeff', drawcoeff)
	ig.igInputFloat('veldecay', veldecay)
	ig.igInputFloat('posdecay', posdecay)
	ig.igInputFloat('repel', repel)
	ig.igInputFloat('restdist', restdist)
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

function App:event(event, ...)
	App.super.event(self, event, ...)
	local canHandleMouse = not ig.igGetIO()[0].WantCaptureMouse
	local canHandleKeyboard = not ig.igGetIO()[0].WantCaptureKeyboard
	
	if canHandleKeyboard then
		if event.type == sdl.SDL_KEYDOWN then
			if event.key.keysym.sym == sdl.SDLK_SPACE then
				running[0] = not running[0]
			end
		end
	end
end

return App
