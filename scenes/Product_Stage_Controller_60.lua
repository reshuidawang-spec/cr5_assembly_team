sim = require('sim')

-- =========================================================
-- Product_Stage_Controller_60.lua
--
-- 作用：
-- 让模型按工艺步骤逐步显示，而不是一开始全部出现。
--
-- 重要逻辑：
-- 模型可以提前建在场景树里，但默认隐藏。
-- 上一步完成后，通过 signal 显示下一步对应模型。
--
-- 使用：
-- 1. 新建 Dummy：Product_Stage_Controller_60
-- 2. 添加 Non-threaded child script
-- 3. 粘贴本脚本
-- 4. 这个脚本不要删除，后续流程控制时要一直保留
--
-- 本脚本不做：
-- 1. 不创建模型
-- 2. 不删除模型
-- 3. 不移动机械臂
-- 4. 不控制夹爪
--
-- 本地测试命令：
-- sim.setStringSignal('cell_product_state','reset')
-- sim.setStringSignal('cell_product_state','assembly_shell')
-- sim.setStringSignal('cell_product_state','assembly_pcb')
-- sim.setStringSignal('cell_product_state','assembly_module')
-- sim.setStringSignal('cell_product_state','assembly_full')
-- sim.setStringSignal('cell_product_state','inspection_full')
-- sim.setStringSignal('cell_product_state','camera_good')
-- sim.setStringSignal('cell_product_state','camera_defect')
-- sim.setStringSignal('cell_conveyor_state','good')
-- sim.setStringSignal('cell_conveyor_state','defect')
-- =========================================================

local RESET_ON_START = true
local CONVEYOR_ENABLED = true

local PRODUCT_ON_BELT_Z = 0.270
local goodStart   = { 0.65, -1.10, PRODUCT_ON_BELT_Z}
local goodEnd     = { 0.65, -2.20, PRODUCT_ON_BELT_Z}
local defectStart = {-0.35, -1.12, PRODUCT_ON_BELT_Z}
local defectEnd   = {-1.45, -1.12, PRODUCT_ON_BELT_Z}
local conveyorSpeed = 0.18

local activeProduct = -1
local activeConveyor = nil
local conveyorStartTime = 0.0

local COLOR_CAMERA_VIEW = {0.75, 0.92, 1.00}
local COLOR_GOOD = {0.20, 0.90, 0.25}
local COLOR_DEFECT = {0.95, 0.15, 0.10}

local function safeGet(path)
    local ok,h = pcall(sim.getObject,path)
    if ok then return h end
    return -1
end

local function setWorldPosition(h,p)
    if h == -1 then return end
    pcall(sim.setObjectPosition,h,-1,p)
end

local function getTreeObjects(path)
    local h = safeGet(path)
    if h == -1 then return {} end

    local result = {h}
    local ok,objs = pcall(sim.getObjectsInTree,h,sim.handle_all,0)

    if ok and objs then
        for i=1,#objs do
            table.insert(result,objs[i])
        end
    end

    return result
end

local function setTreeVisible(path,visible)
    local objs = getTreeObjects(path)
    local layer = visible and 1 or 0
    local count = 0

    for i=1,#objs do
        local h = objs[i]
        local okType,t = pcall(sim.getObjectType,h)

        if okType and t == sim.object_shape_type then
            pcall(sim.setObjectInt32Param,h,sim.objintparam_visibility_layer,layer)
            count = count + 1
        end
    end

    return count
end

local function setTreeColor(path,color)
    local objs = getTreeObjects(path)

    for i=1,#objs do
        local h = objs[i]
        local okType,t = pcall(sim.getObjectType,h)

        if okType and t == sim.object_shape_type then
            pcall(sim.setShapeColor,h,nil,sim.colorcomponent_ambient_diffuse,color)
        end
    end
end

local function hideAllProductStages()
    setTreeVisible('/FiveCR5A_Cell/Parts/Assembly_ControlBox_Product', false)
    setTreeVisible('/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product', false)
end

local function showSupplyAll()
    setTreeVisible('/FiveCR5A_Cell/Parts/Box_Blank', true)
    setTreeVisible('/FiveCR5A_Cell/Parts/PCB_Supply', true)
    setTreeVisible('/FiveCR5A_Cell/Parts/Control_Module_Supply', true)
    setTreeVisible('/FiveCR5A_Cell/Parts/Terminal_Block_Supply', true)
end

local function setProductStage(productName,stage)
    -- stage:
    -- 0 = 全隐藏
    -- 1 = 只显示箱体
    -- 2 = 箱体 + PCB
    -- 3 = 箱体 + PCB + 控制模块
    -- 4 = 完整产品
    local productRoot = '/FiveCR5A_Cell/Parts/' .. productName
    local base = productRoot .. '/' .. productName

    if stage <= 0 then
        setTreeVisible(productRoot,false)
        return
    end

    -- 先隐藏整套产品
    setTreeVisible(productRoot,false)

    -- 再按阶段显示
    setTreeVisible(base .. '_Shell', stage >= 1)
    setTreeVisible(base .. '_PCB', stage >= 2)
    setTreeVisible(base .. '_Control_Module', stage >= 3)
    setTreeVisible(base .. '_Terminal_Block', stage >= 4)
end

local function resetInitialState()
    showSupplyAll()
    setProductStage('Assembly_ControlBox_Product',0)
    setProductStage('Inspection_ControlBox_Product',0)

    setTreeColor('/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area',COLOR_CAMERA_VIEW)

    activeProduct = -1
    activeConveyor = nil
    conveyorStartTime = sim.getSimulationTime()

    sim.clearStringSignal('cell_product_state')
    sim.clearStringSignal('cell_conveyor_state')

    print('[STAGE] reset: only supply parts visible; assembly/inspection products hidden.')
end

local function handleProductState(state)
    if not state then return end

    if state == 'reset' then
        resetInitialState()

    elseif state == 'assembly_shell' or state == 'box_placed' then
        -- R1 或 R3 把箱体放到装配区后
        setTreeVisible('/FiveCR5A_Cell/Parts/Box_Blank',false)
        setProductStage('Assembly_ControlBox_Product',1)
        print('[STAGE] assembly_shell')

    elseif state == 'assembly_pcb' or state == 'pcb_placed' then
        -- R2 把 PCB 放入箱体后
        setTreeVisible('/FiveCR5A_Cell/Parts/PCB_Supply',false)
        setProductStage('Assembly_ControlBox_Product',2)
        print('[STAGE] assembly_pcb')

    elseif state == 'assembly_module' or state == 'module_placed' then
        -- R1/R3 放入控制模块后
        setTreeVisible('/FiveCR5A_Cell/Parts/Control_Module_Supply',false)
        setProductStage('Assembly_ControlBox_Product',3)
        print('[STAGE] assembly_module')

    elseif state == 'assembly_full' or state == 'terminal_placed' then
        -- R1 放入端子排后，装配完整
        setTreeVisible('/FiveCR5A_Cell/Parts/Terminal_Block_Supply',false)
        setProductStage('Assembly_ControlBox_Product',4)
        print('[STAGE] assembly_full')

    elseif state == 'inspection_full' or state == 'product_to_inspection' then
        -- R3 把装配体搬到检测/锁付区
        setProductStage('Assembly_ControlBox_Product',0)
        setProductStage('Inspection_ControlBox_Product',4)
        print('[STAGE] inspection_full')

    elseif state == 'camera_good' then
        setTreeColor('/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area',COLOR_GOOD)
        print('[STAGE] camera_good')

    elseif state == 'camera_defect' then
        setTreeColor('/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area',COLOR_DEFECT)
        print('[STAGE] camera_defect')
    end
end

local function handleConveyorState(state)
    if not state then return end

    local product = safeGet('/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product')
    if product == -1 then
        print('[CONVEYOR WARN] Inspection_ControlBox_Product not found.')
        return
    end

    if state == 'good' then
        setProductStage('Inspection_ControlBox_Product',4)
        setWorldPosition(product,goodStart)
        activeProduct = product
        activeConveyor = 'good'
        conveyorStartTime = sim.getSimulationTime()
        print('[CONVEYOR] good started')

    elseif state == 'defect' then
        setProductStage('Inspection_ControlBox_Product',4)
        setWorldPosition(product,defectStart)
        activeProduct = product
        activeConveyor = 'defect'
        conveyorStartTime = sim.getSimulationTime()
        print('[CONVEYOR] defect started')
    end
end

local function moveAlongLine(obj,p0,p1,speed)
    if obj == -1 then return end

    local t = sim.getSimulationTime() - conveyorStartTime

    local dx = p1[1] - p0[1]
    local dy = p1[2] - p0[2]
    local dz = p1[3] - p0[3]

    local dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist < 0.0001 then return end

    local moveTime = dist / speed
    local a = t / moveTime
    if a > 1 then a = 1 end

    local p = {
        p0[1] + dx*a,
        p0[2] + dy*a,
        p0[3] + dz*a
    }

    setWorldPosition(obj,p)
end

function sysCall_init()
    print('===== Product Stage Controller 60% =====')

    if RESET_ON_START then
        resetInitialState()
    end

    print('[INFO] Use cell_product_state signal to advance stages.')
end

function sysCall_actuation()
    local state = sim.getStringSignal('cell_product_state')
    if state then
        handleProductState(state)
        sim.clearStringSignal('cell_product_state')
    end

    local conveyorState = sim.getStringSignal('cell_conveyor_state')
    if conveyorState then
        handleConveyorState(conveyorState)
        sim.clearStringSignal('cell_conveyor_state')
    end

    if CONVEYOR_ENABLED and activeProduct ~= -1 then
        if activeConveyor == 'good' then
            moveAlongLine(activeProduct,goodStart,goodEnd,conveyorSpeed)
        elseif activeConveyor == 'defect' then
            moveAlongLine(activeProduct,defectStart,defectEnd,conveyorSpeed)
        end
    end
end

function sysCall_cleanup()
end
