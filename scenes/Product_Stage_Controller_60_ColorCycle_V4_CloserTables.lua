sim = require('sim')

-- =========================================================
-- Product_Stage_Controller_60_ColorCycle_V4_CloserTables.lua
--
-- 作用：
-- 1. 让模型按工艺步骤逐步显示；
-- 2. 每开始下一个电柜时，所有电柜相关元件按三套颜色循环；
-- 3. 配合右侧工作台右移后的新布局。
--
-- 使用：
-- 替换旧的 Product_Stage_Controller_60.lua。
-- 这个脚本需要一直启用。
--
-- 颜色逻辑：
-- RESET_CELL / cell_product_state='reset' 时，颜色自动切换到下一套。
-- 三套颜色循环：
--   1. 蓝色系
--   2. 橙色系
--   3. 紫绿色系
--
-- 也可以手动控制：
--   sim.setStringSignal('cell_product_state','color_next')
--   sim.setStringSignal('cell_product_state','color_1')
--   sim.setStringSignal('cell_product_state','color_2')
--   sim.setStringSignal('cell_product_state','color_3')
-- =========================================================

local RESET_ON_START = true
local ADVANCE_COLOR_ON_RESET = true
local CONVEYOR_ENABLED = true

-- 右侧工作台右移后的输送带产品起止点
local PRODUCT_ON_BELT_Z = 0.270
local goodStart   = { 0.85, -1.10, PRODUCT_ON_BELT_Z}
local goodEnd     = { 0.85, -2.20, PRODUCT_ON_BELT_Z}
local defectStart = {-0.15, -1.12, PRODUCT_ON_BELT_Z}
local defectEnd   = {-1.25, -1.12, PRODUCT_ON_BELT_Z}
local conveyorSpeed = 0.18

local activeProduct = -1
local activeConveyor = nil
local conveyorStartTime = 0.0

local COLOR_CAMERA_VIEW = {0.75, 0.92, 1.00}
local COLOR_GOOD = {0.20, 0.90, 0.25}
local COLOR_DEFECT = {0.95, 0.15, 0.10}

local colorIndex = 0

local COLOR_SCHEMES = {
    {
        name='BLUE',
        shell={0.22,0.46,0.92},
        pcb={0.08,0.70,0.95},
        module={0.08,0.22,0.60},
        terminal={0.50,0.85,1.00},
        detail={0.92,0.96,1.00}
    },
    {
        name='ORANGE',
        shell={0.90,0.42,0.16},
        pcb={0.95,0.62,0.18},
        module={0.68,0.22,0.10},
        terminal={1.00,0.82,0.30},
        detail={1.00,0.92,0.82}
    },
    {
        name='PURPLE_GREEN',
        shell={0.50,0.32,0.88},
        pcb={0.20,0.80,0.42},
        module={0.32,0.20,0.62},
        terminal={0.66,0.92,0.34},
        detail={0.92,0.88,1.00}
    }
}

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

local function getScheme()
    if colorIndex < 1 or colorIndex > #COLOR_SCHEMES then
        colorIndex = 1
    end
    return COLOR_SCHEMES[colorIndex]
end

local function applyColorSchemeToProduct(productName)
    local s = getScheme()
    local root = '/FiveCR5A_Cell/Parts/'..productName
    local base = root..'/'..productName

    setTreeColor(base..'_Shell',s.shell)
    setTreeColor(base..'_PCB',s.pcb)
    setTreeColor(base..'_Control_Module',s.module)
    setTreeColor(base..'_Terminal_Block',s.terminal)

    -- 螺钉、标签、插槽等细节不强制全改，避免失去结构感。
end

local function applyColorSchemeToSupply()
    local s = getScheme()

    setTreeColor('/FiveCR5A_Cell/Parts/Box_Blank',s.shell)
    setTreeColor('/FiveCR5A_Cell/Parts/PCB_Supply',s.pcb)
    setTreeColor('/FiveCR5A_Cell/Parts/Control_Module_Supply',s.module)
    setTreeColor('/FiveCR5A_Cell/Parts/Terminal_Block_Supply',s.terminal)

    applyColorSchemeToProduct('Assembly_ControlBox_Product')
    applyColorSchemeToProduct('Inspection_ControlBox_Product')

    print('[COLOR] cabinet color scheme = '..s.name..' index='..tostring(colorIndex))
end

local function nextColor()
    colorIndex = colorIndex + 1
    if colorIndex > #COLOR_SCHEMES then colorIndex = 1 end
    sim.setInt32Signal('cabinet_color_index',colorIndex)
    applyColorSchemeToSupply()
end

local function setColorIndex(idx)
    colorIndex = idx
    if colorIndex < 1 then colorIndex = 1 end
    if colorIndex > #COLOR_SCHEMES then colorIndex = #COLOR_SCHEMES end
    sim.setInt32Signal('cabinet_color_index',colorIndex)
    applyColorSchemeToSupply()
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

    applyColorSchemeToProduct(productName)

    setTreeVisible(productRoot,false)

    setTreeVisible(base .. '_Shell', stage >= 1)
    setTreeVisible(base .. '_PCB', stage >= 2)
    setTreeVisible(base .. '_Control_Module', stage >= 3)
    setTreeVisible(base .. '_Terminal_Block', stage >= 4)
end

local function resetInitialState()
    if ADVANCE_COLOR_ON_RESET then
        nextColor()
    else
        applyColorSchemeToSupply()
    end

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

    elseif state == 'color_next' then
        nextColor()

    elseif state == 'color_1' then
        setColorIndex(1)

    elseif state == 'color_2' then
        setColorIndex(2)

    elseif state == 'color_3' then
        setColorIndex(3)

    elseif state == 'assembly_shell' or state == 'box_placed' then
        setTreeVisible('/FiveCR5A_Cell/Parts/Box_Blank',false)
        setProductStage('Assembly_ControlBox_Product',1)
        print('[STAGE] assembly_shell')

    elseif state == 'assembly_pcb' or state == 'pcb_placed' then
        setTreeVisible('/FiveCR5A_Cell/Parts/PCB_Supply',false)
        setProductStage('Assembly_ControlBox_Product',2)
        print('[STAGE] assembly_pcb')

    elseif state == 'assembly_module' or state == 'module_placed' then
        setTreeVisible('/FiveCR5A_Cell/Parts/Control_Module_Supply',false)
        setProductStage('Assembly_ControlBox_Product',3)
        print('[STAGE] assembly_module')

    elseif state == 'assembly_full' or state == 'terminal_placed' then
        setTreeVisible('/FiveCR5A_Cell/Parts/Terminal_Block_Supply',false)
        setProductStage('Assembly_ControlBox_Product',4)
        print('[STAGE] assembly_full')

    elseif state == 'inspection_full' or state == 'product_to_inspection' then
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
    print('===== Product Stage Controller 60% ColorCycle V4 Closer Tables =====')

    local saved = sim.getInt32Signal('cabinet_color_index')
    if saved and saved >= 1 and saved <= #COLOR_SCHEMES then
        colorIndex = saved
    else
        colorIndex = 0
    end

    if RESET_ON_START then
        resetInitialState()
    else
        if colorIndex == 0 then colorIndex = 1 end
        applyColorSchemeToSupply()
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
