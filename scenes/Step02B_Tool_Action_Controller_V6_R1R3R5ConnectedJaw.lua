sim = require('sim')

-- =========================================================
-- Step02B_Tool_Action_Controller_V5_R1R3R5ConnectedJaw.lua
--
-- 适配当前末端工具：
-- R1：宽口可调夹爪，同时夹箱体和端子排
-- R3：宽口夹爪，夹箱体/完整装配体
-- R5：宽口夹爪，夹检测后的完整产品
-- R2：吸盘
-- R4：螺丝刀
--
-- 关键逻辑：
-- 同一个 R1 夹爪，根据不同对象闭合到不同间隙：
-- R1_ATTACH_BOX       -> 箱体夹持间隙 150 mm
-- R1_ATTACH_TERMINAL  -> 端子排夹持间隙 30 mm
-- R1_ATTACH_MODULE    -> 控制模块夹持间隙 45 mm
--
-- 注意：
-- 这仍然是“工艺仿真夹取”：视觉开合 + attach/detach 绑定。
-- =========================================================

local CMD_SIGNAL = 'tool_cmd'
local SNAP_TO_TCP_ON_ATTACH = true
local SCREW_AUTO_TIME = 3.0

-- gap = 两个夹持垫内侧之间的目标距离，单位 m
local gripperCfg = {
    R1 = {
        tool='R1T',
        left='R1T_left_finger_link',
        right='R1T_right_finger_link',
        padY=0.020,
        openGap=0.170,
        defaultCloseGap=0.150,
        boxGap=0.150,
        terminalGap=0.030,
        moduleGap=0.045,
    },
    R3 = {
        tool='R3T',
        left='R3T_left_finger_link',
        right='R3T_right_finger_link',
        padY=0.020,
        openGap=0.170,
        defaultCloseGap=0.150,
        boxGap=0.150,
    },
    R5 = {
        tool='R5T',
        left='R5T_left_finger_link',
        right='R5T_right_finger_link',
        padY=0.020,
        openGap=0.170,
        defaultCloseGap=0.150,
        boxGap=0.150,
    },
}

local tcpNames = {
    R1 = 'R1T_tool_tcp',
    R2 = 'R2T_tool_tcp',
    R3 = 'R3T_tool_tcp',
    R4 = 'R4T_tool_tcp',
    R5 = 'R5T_tool_tcp',
}

local partPaths = {
    BOX = '/FiveCR5A_Cell/Parts/Box_Blank',
    PCB = '/FiveCR5A_Cell/Parts/PCB_Supply',
    MODULE = '/FiveCR5A_Cell/Parts/Control_Module_Supply',
    TERMINAL = '/FiveCR5A_Cell/Parts/Terminal_Block_Supply',
    ASSEMBLY_PRODUCT = '/FiveCR5A_Cell/Parts/Assembly_ControlBox_Product',
    INSPECTION_PRODUCT = '/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product',
}

local attached = {R1=-1, R2=-1, R3=-1, R5=-1}
local fingerHome = {}

local r4SpinName = 'R4T_screw_spin_link'
local r4Spinning = false
local r4Angle = 0
local r4StartTime = 0
local r4Speed = 20.0
local screwHead = -1
local screwAngle = 0

local function log(msg)
    print('[TOOL CTRL] '..msg)
end

local function safeGet(path)
    local ok,h = pcall(sim.getObject,path)
    if ok then return h end
    return -1
end

local function names(h)
    local t={}
    local ok,n=pcall(sim.getObjectAlias,h,0); if ok and n then t[#t+1]=n end
    local ok2,n2=pcall(sim.getObjectAlias,h,1); if ok2 and n2 then t[#t+1]=n2 end
    local ok3,n3=pcall(sim.getObjectName,h); if ok3 and n3 then t[#t+1]=n3 end
    return t
end

local function exactName(h,target)
    for _,n in ipairs(names(h)) do
        if n == target then return true end
    end
    return false
end

local function match(h,target)
    for _,n in ipairs(names(h)) do
        if n == target or string.find(n,target,1,true) then return true end
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

local function findContainsAnywhere(target)
    local ok,objs=pcall(sim.getObjects,sim.handle_all)
    if ok and objs then
        for _,o in ipairs(objs) do
            if match(o,target) then return o end
        end
    end
    return -1
end

local function setTreeVisible(root,visible)
    if root == -1 then return end
    local ok,objs=pcall(sim.getObjectsInTree,root,sim.handle_all,0)
    if not ok or not objs then return end

    local layer = visible and 1 or 0
    for _,o in ipairs(objs) do
        local okT,t=pcall(sim.getObjectType,o)
        if okT and t==sim.object_shape_type then
            pcall(sim.setObjectInt32Param,o,sim.objintparam_visibility_layer,layer)
        end
    end
end

local function cacheFingerHome(robotName)
    local cfg = gripperCfg[robotName]
    if not cfg then return end

    local tool = findExactAnywhere(cfg.tool)
    local left = findExactAnywhere(cfg.left)
    local right = findExactAnywhere(cfg.right)

    if tool == -1 or left == -1 or right == -1 then
        log('WARN: cannot cache finger home for '..robotName)
        return
    end

    local lp = sim.getObjectPosition(left,tool)
    local rp = sim.getObjectPosition(right,tool)

    fingerHome[robotName] = {
        left = {lp[1], lp[2], lp[3]},
        right = {rp[1], rp[2], rp[3]},
    }

    log(string.format('%s home cached: L_y=%.4f R_y=%.4f',robotName,lp[2],rp[2]))
end

local function cacheAllFingerHome()
    cacheFingerHome('R1')
    cacheFingerHome('R3')
    cacheFingerHome('R5')
end

local function setFingerGap(robotName,gap)
    local cfg = gripperCfg[robotName]
    if not cfg then
        log('ERROR: no gripper config for '..robotName)
        return
    end

    local tool = findExactAnywhere(cfg.tool)
    local left = findExactAnywhere(cfg.left)
    local right = findExactAnywhere(cfg.right)

    if tool == -1 or left == -1 or right == -1 then
        log('ERROR: gripper objects not found for '..robotName)
        return
    end

    if not fingerHome[robotName] then
        cacheFingerHome(robotName)
    end
    if not fingerHome[robotName] then
        log('ERROR: no finger home for '..robotName)
        return
    end

    -- targetY 是夹指中心到中线的距离：
    -- 内侧间距 gap = 2*targetY - padY
    local targetY = (gap + cfg.padY) / 2.0
    local home = fingerHome[robotName]

    local lp = {home.left[1], targetY, home.left[3]}
    local rp = {home.right[1], -targetY, home.right[3]}

    sim.setObjectPosition(left,tool,lp)
    sim.setObjectPosition(right,tool,rp)

    log(string.format('%s_SET_GAP %.0f mm',robotName,gap*1000))
end

local function setFingerOpen(robotName)
    setFingerGap(robotName, gripperCfg[robotName].openGap)
end

local function setFingerClose(robotName)
    setFingerGap(robotName, gripperCfg[robotName].defaultCloseGap)
end

local function getPart(key)
    local path = partPaths[key]
    if not path then return -1 end
    return safeGet(path)
end

local function attachTo(robotName,partKey,gap)
    local obj = getPart(partKey)
    local tcp = findExactAnywhere(tcpNames[robotName])

    if obj == -1 then
        log('ERROR: part not found: '..partKey)
        return
    end
    if tcp == -1 then
        log('ERROR: TCP not found for '..robotName)
        return
    end

    if robotName == 'R1' or robotName == 'R3' or robotName == 'R5' then
        setFingerGap(robotName,gap or gripperCfg[robotName].defaultCloseGap)
    end

    setTreeVisible(obj,true)
    sim.setObjectParent(obj,tcp,true)

    if SNAP_TO_TCP_ON_ATTACH then
        sim.setObjectPosition(obj,tcp,{0,0,0})
        sim.setObjectOrientation(obj,tcp,{0,0,0})
    end

    attached[robotName] = obj
    log('ATTACH '..partKey..' -> '..robotName)
end

local function detachFrom(robotName)
    local obj = attached[robotName]
    if obj == nil or obj == -1 then
        log('WARN: '..robotName..' has no attached object')
        return -1
    end

    local partsRoot = safeGet('/FiveCR5A_Cell/Parts')
    sim.setObjectParent(obj,partsRoot,true)
    attached[robotName] = -1
    log('DETACH from '..robotName)
    return obj
end

local function releaseSupplyWithStage(robotName,stageSignal)
    local obj = detachFrom(robotName)
    if obj ~= -1 then setTreeVisible(obj,false) end

    if robotName == 'R1' or robotName == 'R3' or robotName == 'R5' then
        setFingerOpen(robotName)
    end

    if stageSignal then
        sim.setStringSignal('cell_product_state',stageSignal)
        log('STAGE -> '..stageSignal)
    end
end

local function releaseProductToInspection()
    local obj = detachFrom('R3')
    if obj ~= -1 then setTreeVisible(obj,false) end
    setFingerOpen('R3')
    sim.setStringSignal('cell_product_state','inspection_full')
    log('STAGE -> inspection_full')
end

local function releaseProductToConveyor(kind)
    local obj = detachFrom('R5')
    if obj ~= -1 then setTreeVisible(obj,false) end
    setFingerOpen('R5')

    if kind == 'good' then
        sim.setStringSignal('cell_conveyor_state','good')
        log('CONVEYOR -> good')
    elseif kind == 'defect' then
        sim.setStringSignal('cell_conveyor_state','defect')
        log('CONVEYOR -> defect')
    end
end

local function findScrewHead()
    local h = safeGet('/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product/Inspection_ControlBox_Product_Terminal_Block/Inspection_ControlBox_Product_Terminal_Block_Main_ScrewHead')
    if h ~= -1 then return h end

    h = findContainsAnywhere('Main_ScrewHead')
    if h ~= -1 then return h end

    return findContainsAnywhere('ScrewHead')
end

local function r4Start()
    local spin = findExactAnywhere(r4SpinName)
    if spin == -1 then
        log('ERROR: R4 screw spin link not found: '..r4SpinName)
        return
    end

    screwHead = findScrewHead()
    r4Spinning = true
    r4StartTime = sim.getSimulationTime()
    log('R4_SCREW_START')
end

local function r4Stop()
    r4Spinning = false
    log('R4_SCREW_STOP')
end

local function handle(cmd)
    if not cmd or cmd == '' then return end

    if cmd == 'R1_GRIPPER_OPEN' then setFingerOpen('R1')
    elseif cmd == 'R1_GRIPPER_CLOSE' then setFingerClose('R1')
    elseif cmd == 'R1_GRIPPER_CLOSE_BOX' then setFingerGap('R1',gripperCfg.R1.boxGap)
    elseif cmd == 'R1_GRIPPER_CLOSE_TERMINAL' then setFingerGap('R1',gripperCfg.R1.terminalGap)
    elseif cmd == 'R1_GRIPPER_CLOSE_MODULE' then setFingerGap('R1',gripperCfg.R1.moduleGap)

    elseif cmd == 'R3_GRIPPER_OPEN' then setFingerOpen('R3')
    elseif cmd == 'R3_GRIPPER_CLOSE' then setFingerClose('R3')

    elseif cmd == 'R5_GRIPPER_OPEN' then setFingerOpen('R5')
    elseif cmd == 'R5_GRIPPER_CLOSE' then setFingerClose('R5')

    elseif cmd == 'ALL_GRIPPERS_OPEN' then
        setFingerOpen('R1'); setFingerOpen('R3'); setFingerOpen('R5')
    elseif cmd == 'ALL_GRIPPERS_CLOSE' then
        setFingerClose('R1'); setFingerClose('R3'); setFingerClose('R5')

    -- R1：既能夹箱体，也能夹端子排/模块
    elseif cmd == 'R1_ATTACH_BOX' then attachTo('R1','BOX',gripperCfg.R1.boxGap)
    elseif cmd == 'R1_RELEASE_BOX_ASSEMBLY' then releaseSupplyWithStage('R1','assembly_shell')
    elseif cmd == 'R1_ATTACH_MODULE' then attachTo('R1','MODULE',gripperCfg.R1.moduleGap)
    elseif cmd == 'R1_RELEASE_MODULE_ASSEMBLY' then releaseSupplyWithStage('R1','assembly_module')
    elseif cmd == 'R1_ATTACH_TERMINAL' then attachTo('R1','TERMINAL',gripperCfg.R1.terminalGap)
    elseif cmd == 'R1_RELEASE_TERMINAL_ASSEMBLY' then releaseSupplyWithStage('R1','assembly_full')

    -- R3：仍然保留抓箱体/完整产品的能力
    elseif cmd == 'R3_ATTACH_BOX' then attachTo('R3','BOX',gripperCfg.R3.boxGap)
    elseif cmd == 'R3_RELEASE_BOX_ASSEMBLY' then releaseSupplyWithStage('R3','assembly_shell')
    elseif cmd == 'R3_ATTACH_ASSEMBLY_PRODUCT' then attachTo('R3','ASSEMBLY_PRODUCT',gripperCfg.R3.boxGap)
    elseif cmd == 'R3_RELEASE_PRODUCT_INSPECTION' then releaseProductToInspection()

    -- R2：PCB
    elseif cmd == 'R2_VACUUM_ON' then log('R2_VACUUM_ON')
    elseif cmd == 'R2_VACUUM_OFF' then log('R2_VACUUM_OFF')
    elseif cmd == 'R2_ATTACH_PCB' then attachTo('R2','PCB')
    elseif cmd == 'R2_RELEASE_PCB_ASSEMBLY' then releaseSupplyWithStage('R2','assembly_pcb')

    -- R4：螺丝刀
    elseif cmd == 'R4_SCREW_START' then r4Start()
    elseif cmd == 'R4_SCREW_STOP' then r4Stop()

    -- R5：分拣
    elseif cmd == 'R5_ATTACH_INSPECTION_PRODUCT' then attachTo('R5','INSPECTION_PRODUCT',gripperCfg.R5.boxGap)
    elseif cmd == 'R5_RELEASE_GOOD' then releaseProductToConveyor('good')
    elseif cmd == 'R5_RELEASE_DEFECT' then releaseProductToConveyor('defect')

    else
        log('UNKNOWN CMD: '..cmd)
    end
end

function sysCall_init()
    print('===== Step02B Tool Action Controller V5 R1 Box+Terminal =====')
    print('[INFO] Keep this script enabled.')
    print('[INFO] Command signal: '..CMD_SIGNAL)

    cacheAllFingerHome()

    setFingerOpen('R1')
    setFingerOpen('R3')
    setFingerOpen('R5')

    sim.clearStringSignal(CMD_SIGNAL)
end

function sysCall_actuation()
    local cmd = sim.getStringSignal(CMD_SIGNAL)
    if cmd and cmd ~= '' then
        handle(cmd)
        sim.clearStringSignal(CMD_SIGNAL)
    end

    if r4Spinning then
        local dt = sim.getSimulationTimeStep()
        local spin = findExactAnywhere(r4SpinName)

        if spin ~= -1 then
            local parent = sim.getObjectParent(spin)
            r4Angle = r4Angle + r4Speed * dt
            if r4Angle > math.pi*2 then r4Angle = r4Angle - math.pi*2 end
            sim.setObjectOrientation(spin,parent,{0,0,r4Angle})
        end

        if screwHead ~= -1 then
            local parent = sim.getObjectParent(screwHead)
            screwAngle = screwAngle + r4Speed * dt
            if screwAngle > math.pi*2 then screwAngle = screwAngle - math.pi*2 end
            pcall(sim.setObjectOrientation,screwHead,parent,{0,0,screwAngle})
        end

        if sim.getSimulationTime() - r4StartTime > SCREW_AUTO_TIME then
            r4Spinning = false
            sim.setStringSignal('cell_product_state','screw_done')
            log('R4_SCREW_AUTO_DONE')
        end
    end
end

function sysCall_cleanup()
end
