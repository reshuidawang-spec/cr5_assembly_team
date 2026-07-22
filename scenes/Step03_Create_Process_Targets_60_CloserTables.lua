sim = require('sim')

-- =========================================================
-- Step03_Create_Process_Targets_60_CloserTables.lua
--
-- 第 3 步：创建 R1~R5 工艺目标点 APP/TCP
--
-- 作用：
-- 给后续路径规划/控制成员提供标准目标点。
--
-- 命名规则：
-- APP = Approach，接近点，位于目标点上方；
-- TCP = 实际抓取/放置/操作点；
-- PRESS = R4 螺丝刀下压点。
--
-- 使用：
-- 1. 确认 Step01 场景、Step02 末端工具都已经完成；
-- 2. 新建 Dummy：Step03_Create_Process_Targets_60；
-- 3. 添加 Non-threaded child script；
-- 4. 粘贴本脚本；
-- 5. 运行一次；
-- 6. 成功后禁用或删除本脚本；
-- 7. 后续如果点位不合适，直接在场景里拖动 Dummy 微调即可。
--
-- 注意：
-- 这些是初始合理点位，不是最终避障路径。
-- 后续路径规划成员只需要读取这些 Dummy 的世界位姿。
-- =========================================================

local RECREATE_TARGETS = true
local SHOW_TARGETS = true

-- 高度参数
local TABLE_TOP_Z = 0.120
local AREA_H = 0.035
local FIXTURE_H = 0.060
local FIXTURE_TOP_Z = TABLE_TOP_Z + AREA_H + FIXTURE_H -- 0.215
local PRODUCT_BOTTOM_Z = FIXTURE_TOP_Z + 0.001       -- 0.216

-- 60% 工件尺寸参考
local BOX_H = 0.072
local PCB_H = 0.0048
local MODULE_H = 0.021
local TERMINAL_H = 0.021

-- 常用操作高度
local BOX_CENTER_Z = PRODUCT_BOTTOM_Z + BOX_H/2       -- 0.252
local BOX_TOP_Z = PRODUCT_BOTTOM_Z + BOX_H            -- 0.288
local PCB_PICK_Z = PRODUCT_BOTTOM_Z + 0.020
local MODULE_PICK_Z = PRODUCT_BOTTOM_Z + 0.055
local TERMINAL_PICK_Z = PRODUCT_BOTTOM_Z + 0.050

local APP_LIFT = 0.180
local SCREW_APP_LIFT = 0.160

-- 和 Step01 60% 场景一致的平面坐标
local P = {
    boxSupply      = {-1.86,  0.22},
    terminalSupply = {-1.82, -0.02},
    pcbSupply      = {-1.22, -0.42},
    moduleSupply   = {-0.78, -0.20},
    assembly       = {-1.08,  0.12},
    inspection     = { 0.35,  0.05},
    goodPlace      = { 0.85, -1.10},
    defectPlace    = {-0.15, -1.12},
}

local function safeGet(path)
    local ok,h = pcall(sim.getObject,path)
    if ok then return h end
    return -1
end

local function aliases(h)
    local t={}
    local ok,n=pcall(sim.getObjectAlias,h,0)
    if ok and n then t[#t+1]=n end
    local ok2,n2=pcall(sim.getObjectAlias,h,1)
    if ok2 and n2 then t[#t+1]=n2 end
    return t
end

local function exactName(h,target)
    for _,n in ipairs(aliases(h)) do
        if n == target then return true end
    end
    return false
end

local function findExactAnywhere(target)
    local h=safeGet('/'..target)
    if h~=-1 and exactName(h,target) then return h end

    local ok,objs=pcall(sim.getObjects,sim.handle_all)
    if ok and objs then
        for _,o in ipairs(objs) do
            if exactName(o,target) then return o end
        end
    end
    return -1
end

local function removeTree(h)
    if h == -1 then return end

    local ok,objs = pcall(sim.getObjectsInTree,h,sim.handle_all,0)
    if ok and objs then
        sim.removeObjects(objs)
    else
        sim.removeObjects({h})
    end
end

local function ensureGroup(parent,name)
    local old = safeGet('/FiveCR5A_Cell/Targets/'..name)
    if old ~= -1 and RECREATE_TARGETS then
        removeTree(old)
    elseif old ~= -1 then
        return old
    end

    local d = sim.createDummy(0.025)
    sim.setObjectAlias(d,name)
    sim.setObjectParent(d,parent,true)
    sim.setObjectPosition(d,parent,{0,0,0})
    sim.setObjectOrientation(d,parent,{0,0,0})
    return d
end

local function colorDummy(h,color)
    pcall(sim.setObjectColor,h,0,sim.colorcomponent_ambient_diffuse,color)
end

local COLORS = {
    R1 = {1.00,0.30,0.25},
    R2 = {0.25,0.60,1.00},
    R3 = {0.25,1.00,0.35},
    R4 = {1.00,0.75,0.10},
    R5 = {0.95,0.25,1.00},
    SENSOR = {0.50,0.90,1.00},
}

local function makeTarget(group,name,pos,ori,color)
    local old = findExactAnywhere(name)
    if old ~= -1 and RECREATE_TARGETS then
        removeTree(old)
    elseif old ~= -1 then
        return old
    end

    local d = sim.createDummy(0.035)
    sim.setObjectAlias(d,name)
    sim.setObjectParent(d,group,true)
    sim.setObjectPosition(d,-1,pos)
    sim.setObjectOrientation(d,-1,ori or {0,0,0})

    if not SHOW_TARGETS then
        pcall(sim.setObjectInt32Param,d,sim.objintparam_visibility_layer,0)
    end

    colorDummy(d,color)
    return d
end

local function pxy(xy,z)
    return {xy[1],xy[2],z}
end

local function appOf(pos,lift)
    return {pos[1],pos[2],pos[3] + (lift or APP_LIFT)}
end

local function printTarget(name,pos)
    print(string.format('[TARGET] %-36s = {%.3f, %.3f, %.3f}',name,pos[1],pos[2],pos[3]))
end

local function makePair(group,prefix,tcpPos,ori,color,lift)
    local app = appOf(tcpPos,lift or APP_LIFT)

    makeTarget(group,prefix..'_APP',app,ori,color)
    makeTarget(group,prefix..'_TCP',tcpPos,ori,color)

    printTarget(prefix..'_APP',app)
    printTarget(prefix..'_TCP',tcpPos)
end

function sysCall_init()
    print('===== Step03 Create Process Targets 60% Separated Tables =====')

    local cell = safeGet('/FiveCR5A_Cell')
    if cell == -1 then
        print('[ERROR] /FiveCR5A_Cell not found. Run Step01 first.')
        return
    end

    local targets = safeGet('/FiveCR5A_Cell/Targets')
    if targets == -1 then
        targets = sim.createDummy(0.030)
        sim.setObjectAlias(targets,'Targets')
        sim.setObjectParent(targets,cell,true)
        sim.setObjectPosition(targets,cell,{0,0,0})
    end

    local gR1 = ensureGroup(targets,'R1_Targets')
    local gR2 = ensureGroup(targets,'R2_Targets')
    local gR3 = ensureGroup(targets,'R3_Targets')
    local gR4 = ensureGroup(targets,'R4_Targets')
    local gR5 = ensureGroup(targets,'R5_Targets')
    local gS  = ensureGroup(targets,'Sensor_Targets')

    -- HOME_REF：放在各机械臂附近上方，只做参考初始位
    makeTarget(gR1,'R1_HOME_REF',{-1.55, 0.55,0.70},{0,0,0},COLORS.R1)
    makeTarget(gR2,'R2_HOME_REF',{-1.55,-0.20,0.70},{0,0,0},COLORS.R2)
    makeTarget(gR3,'R3_HOME_REF',{-0.60, 0.35,0.70},{0,0,0},COLORS.R3)
    makeTarget(gR4,'R4_HOME_REF',{ 0.75, 0.25,0.70},{0,0,0},COLORS.R4)
    makeTarget(gR5,'R5_HOME_REF',{ 0.35,-0.45,0.70},{0,0,0},COLORS.R5)

    -- R1：箱体上料 + 端子排安装
    -- R1 宽口夹爪既夹箱体，也夹端子排。点位后续可微调。
    makePair(gR1,'R1_BOX_PICK',  pxy(P.boxSupply,BOX_CENTER_Z),     {0,0,0},COLORS.R1)
    makePair(gR1,'R1_BOX_PLACE', pxy(P.assembly,BOX_CENTER_Z),      {0,0,0},COLORS.R1)

    makePair(gR1,'R1_TERMINAL_PICK',  pxy(P.terminalSupply,TERMINAL_PICK_Z), {0,0,0},COLORS.R1)
    makePair(gR1,'R1_TERMINAL_PLACE', {P.assembly[1]+0.020,P.assembly[2]-0.035,BOX_TOP_Z+TERMINAL_H/2}, {0,0,0},COLORS.R1)

    -- R2：PCB 安装
    makePair(gR2,'R2_PCB_PICK',  pxy(P.pcbSupply,PCB_PICK_Z), {0,0,0},COLORS.R2)
    makePair(gR2,'R2_PCB_PLACE', {P.assembly[1],P.assembly[2],BOX_TOP_Z+PCB_H/2}, {0,0,0},COLORS.R2)

    -- R3：控制模块安装 + 完整装配体转移到检测区
    makePair(gR3,'R3_MODULE_PICK',  pxy(P.moduleSupply,MODULE_PICK_Z), {0,0,0},COLORS.R3)
    makePair(gR3,'R3_MODULE_PLACE', {P.assembly[1]-0.025,P.assembly[2]+0.025,BOX_TOP_Z+PCB_H+MODULE_H/2}, {0,0,0},COLORS.R3)

    makePair(gR3,'R3_PRODUCT_PICK', {P.assembly[1],P.assembly[2],BOX_CENTER_Z}, {0,0,0},COLORS.R3)
    makePair(gR3,'R3_PRODUCT_PLACE_INSPECTION', {P.inspection[1],P.inspection[2],BOX_CENTER_Z}, {0,0,0},COLORS.R3)

    -- R4：螺钉锁付
    local screwTcp = {P.inspection[1]+0.020,P.inspection[2]-0.035,BOX_TOP_Z+TERMINAL_H+0.030}
    local screwPress = {screwTcp[1],screwTcp[2],screwTcp[3]-0.030}
    local screwApp = appOf(screwTcp,SCREW_APP_LIFT)

    makeTarget(gR4,'R4_SCREW_APP',screwApp,{0,0,0},COLORS.R4)
    makeTarget(gR4,'R4_SCREW_TCP',screwTcp,{0,0,0},COLORS.R4)
    makeTarget(gR4,'R4_SCREW_PRESS',screwPress,{0,0,0},COLORS.R4)
    printTarget('R4_SCREW_APP',screwApp)
    printTarget('R4_SCREW_TCP',screwTcp)
    printTarget('R4_SCREW_PRESS',screwPress)

    -- R5：检测区产品分拣
    makePair(gR5,'R5_PRODUCT_PICK', pxy(P.inspection,BOX_CENTER_Z), {0,0,0},COLORS.R5)
    makePair(gR5,'R5_GOOD_PLACE',   pxy(P.goodPlace,BOX_CENTER_Z),  {0,0,0},COLORS.R5)
    makePair(gR5,'R5_DEFECT_PLACE', pxy(P.defectPlace,BOX_CENTER_Z),{0,0,0},COLORS.R5)

    -- 相机检测中心
    makeTarget(gS,'CAMERA_INSPECTION_CENTER',{P.inspection[1],P.inspection[2],BOX_TOP_Z+0.120},{0,0,0},COLORS.SENSOR)

    print('===== Step03 Done. Disable this script after success. =====')
    print('[NEXT] Manually fine tune TCP dummies if necessary, then export target list to control members.')
end

function sysCall_cleanup()
end
