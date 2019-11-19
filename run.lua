#!/usr/bin/env luajit
local range = require 'ext.range'
local matrix = require 'matrix'

local function mod1(a,b) return (a-1)%b+1 end

local nodes = range(7)

local weights = matrix.zeros(7,7)
for i=1,7 do
	weights[i][mod1(i+1,7)] = 1
	weights[i][mod1(i+2,7)] = 1
	weights[i][mod1(i+4,7)] = 1
end

require 'force-directed-graph'{
	nodes = nodes,
	weights = weights,
}:run()
