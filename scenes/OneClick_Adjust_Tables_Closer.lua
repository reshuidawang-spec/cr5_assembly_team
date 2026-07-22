sim = require('sim')

-- =========================================================
-- OneClick_Adjust_Tables_Closer.lua
--
-- 作用：
-- 上一版把右侧工作台移到了 x=0.95，两个工作台不重叠，但看起来偏远。
-- 本脚本把右侧工作台调整到 x=0.75 左右：
--
-- 左侧工作台中心约：(-1.20,  0.25)
-- 右侧工作台目标：  ( 0.75, -0.10)
--
-- 两个工作台半径约 0.95 m，中心距离约 1.98 m，
-- 这样两个圆台基本不重叠，同时间距不会太大。
--
-- 使用：
-- 1. 新建 Dummy：OneClick_Adjust_Tables_Closer
-- 2. 添加 Non-threaded child script
-- 3. 删除默认模板
-- 4. 粘贴本脚本
-- 5. 等 Console 出现 DONE
-- 6. 禁用或删除本脚本
-- =========================================================

local executed = false

local TARGET_RIGHT_TABLE_X = 0.75
local TARGET_RIGHT_TABLE_Y = -0.10
local EPS = 0.010

local function log(msg)
    print('[TABLE CLOSER] '..msg)
end

local function safeGet(path)
    local ok,h = pcall(sim.getObject,path)
    if ok then return h end
    return -1
end

local function aliases(h)
    local t = {}
    local ok,a0 = pcall(sim.getObjectAlias,h,0)
    if ok and a0 then table.insert(t,a0) end
    local ok2,a1 = pcall(sim.getObjectAlias,h,1)
    if ok2 and a1 then table.insert(t,a1) end
    local ok3,n = pcall(sim.getObjectName,h)
    if ok3 and n then table.insert(t,n) end
    return t
end

local function exactName(h,target)
    for _,n in ipairs(aliases(h)) do
        if n == target then return true end
    end
    return false
end

local function findExactAnywhere(target)
    local h = safeGet('/'..target)
    if h ~= -1 and exactName(h,target) then return h end

    local ok,objs = pcall(sim.getObjects,sim.handle_all)
    if ok and objs then
        for _,o in ipairs(objs) do
            if exactName(o,target) then return o end
        end
    end
    return -1
end

local function moveByHandle(h,name,dx,dy,dz)
    if h == -1 then
        log('skip, not found: '..name)
        return false
    end

    local p = sim.getObjectPosition(h,-1)
    sim.setObjectPosition(h,-1,{p[1]+dx,p[2]+dy,p[3]+(dz or 0)})
    log(string.format('move %-48s  dx=%.3f  dy=%.3f',name,dx,dy))
    return true
end

local function moveByName(name,dx,dy,dz)
    return moveByHandle(findExactAnywhere(name),name,dx,dy,dz)
end

local function moveByPath(path,dx,dy,dz)
    return moveByHandle(safeGet(path),path,dx,dy,dz)
end

local function doFix()
    if executed then return end
    executed = true

    print('===== OneClick Adjust Tables Closer =====')

    local rightTable = findExactAnywhere('Damping_Table_Right')
    if rightTable == -1 then
        log('ERROR: Damping_Table_Right not found.')
        return
    end

    local p = sim.getObjectPosition(rightTable,-1)
    local dx = TARGET_RIGHT_TABLE_X - p[1]
    local dy = TARGET_RIGHT_TABLE_Y - p[2]

    log(string.format('Right table current: x=%.3f y=%.3f',p[1],p[2]))
    log(string.format('Right table target : x=%.3f y=%.3f',TARGET_RIGHT_TABLE_X,TARGET_RIGHT_TABLE_Y))

    if math.abs(dx) < EPS and math.abs(dy) < EPS then
        log('位置已经合适，不需要移动。')
        log('DONE')
        return
    end

    log(string.format('Apply shift: dx=%.3f dy=%.3f',dx,dy))

    -- 右侧工作台和脚垫
    moveByName('Damping_Table_Right',dx,dy,0)
    for i=1,8 do
        moveByName('Right_RubberPad_'..i,dx,dy,0)
    end

    -- 右侧机械臂和基座
    moveByName('R4_Base',dx,dy,0)
    moveByName('R5_Base',dx,dy,0)
    moveByPath('/R4',dx,dy,0)
    moveByPath('/R5',dx,dy,0)

    -- 检测/锁付区
    moveByName('Inspection_Screw_Area',dx,dy,0)
    moveByName('Inspection_Platform',dx,dy,0)

    -- 相机、检测区产品、传送带
    moveByPath('/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station',dx,dy,0)
    moveByPath('/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product',dx,dy,0)
    moveByPath('/FiveCR5A_Cell/Conveyors/Good_Conveyor',dx,dy,0)
    moveByPath('/FiveCR5A_Cell/Conveyors/Defect_Conveyor',dx,dy,0)

    -- 目标点
    moveByPath('/FiveCR5A_Cell/Targets/R4_Targets',dx,dy,0)
    moveByPath('/FiveCR5A_Cell/Targets/R5_Targets',dx,dy,0)
    moveByName('R3_PRODUCT_PLACE_INSPECTION_APP',dx,dy,0)
    moveByName('R3_PRODUCT_PLACE_INSPECTION_TCP',dx,dy,0)
    moveByName('CAMERA_INSPECTION_CENTER',dx,dy,0)

    log('DONE. 现在可以禁用或删除这个脚本。')
end

function sysCall_init()
    doFix()
end

function sysCall_nonSimulation()
    doFix()
end

function sysCall_cleanup()
end
