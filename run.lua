#!/usr/bin/env luajit
local table = require 'ext.table'
local range = require 'ext.range'
local matrix = require 'matrix'
local vec3d = require 'vec-ffi.vec3d'

local function mod1(a,b) return (a-1)%b+1 end

local nodes = table()
local weights = table()
local function addNode(n, pos)
	local result = nodes:find(nil, function(m) return m.name == n end)
	if result then return result end
	nodes:insert{name=n, pos=pos}
	return #nodes
end
local function addEdge(a,b, v)
	local na = addNode(a)
	local nb = addNode(b)
	v = v or 1
	weights[na] = weights[na] or {}
	weights[nb] = weights[nb] or {}
	weights[na][nb] = v
	weights[nb][na] = v
end


--[[
local n = 10
for i=1,n do
	local j = mod1(i+1,n)
	local v = 1
	addEdge('a'..i, 'b'..i, v)
	addEdge('b'..i, 'b'..j, v)
	addEdge('b'..j, 'a'..j, v)
	addEdge('a'..j, 'a'..i, v)
	addEdge('a'..i, 'b'..j, v)
	addEdge('a'..j, 'b'..i, v)
end
--]]

local n = 10
for i=1,n do
	for j=1,n do
		addNode(i..','..j, vec3d(i-n/2,j-n/2,0) * .1)
	end
end
for i=1,n do
	for j=1,n do
		if i<n then addEdge(i..','..j, (i+1)..','..j, .1) end
		if j<n then addEdge(i..','..j, i..','..(j+1), .1) end
		if i<n and j<n then 
			addEdge(i..','..j, (i+1)..','..(j+1), .1 * math.sqrt(2)) 
			addEdge((i+1)..','..j, i..','..(j+1), .1 * math.sqrt(2)) 
		end
	end
end



local quatInOct = {0,1,3}
local octInSed = {0,1,2,4,5,8,10}

--[[ quaternions
local nodes = range(3)
local weights = matrix.zeros(3,3)
for i=1,3 do
	weights[i][mod1(i+1,3)] = 1
end
--]]

--[[ quaternions in octonions
local function addTriplet(t, a, b, c)
	addEdge(a, b)
	addEdge(b, c)
	addEdge(c, a)
	addEdge(a, t, .1)
	addEdge(b, t, .1)
	addEdge(c, t, .1)
end
addTriplet('t1', 'e1', 'e2', 'e3')
addTriplet('t2', 'e2', 'e4', 'e6')
addTriplet('t4', 'e4', 'e3', '-e7')
addTriplet('t3', 'e3', 'e6', 'e5')
addTriplet('t6', 'e6', '-e7', 'e1')
addTriplet('t7', '-e7', 'e5', 'e2')
addTriplet('t5', 'e5', 'e1', 'e4')
for _,n in ipairs{'e1', 'e2', 'e4', 'e3', 'e6', '-e7', 'e5'} do
	addEdge('center', n, 3)
end
--]]
--[=[
for i=1,7 do
	for j=1,#quatInOct do
		local a = mod1(i + quatInOct[j], 7)
		local b = mod1(i + quatInOct[mod1(j + 1, 3)], 7)
print(a,b)
		weights[a][b] = 1
		weights[b][a] = 1
	end
end
--]=]

--[[ sedenions in quaternions
local nodes = range(15)
local weights = matrix.zeros(15,15)
for i=1,1 do --15 do
	for j=1,#octInSed do
		local e = mod1(i + octInSed[j], 15)
		weights[i][e] = weights[i][e] + 1
if false and i == 1 then		
		for _,k in ipairs(quatInOct) do
			local e = mod1(i + octInSed[mod1(j + quatInOct[k], 7)], 15)
			weights[i][e] = weights[i][e] + 1
		end
end
	end
end
--]]

for i=1,#nodes do
	weights[i] = weights[i] or {}
	for j=1,#nodes do
		weights[i][j] = weights[i][j] or 0
	end
end
return require 'force-directed-graph'{
	nodes = nodes,
	weights = weights,
}:run()
