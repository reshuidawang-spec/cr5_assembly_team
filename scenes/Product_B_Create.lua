-- Product_B_Create.lua
-- 生成B型号产品：从A型号复制所有供料件、装配体、检测体，并差异化着色
-- 使用：新建 Dummy 挂载本脚本，运行一次后禁用
-- 前提：必须先运行 Step01（创建场景和A产品）

sim = require('sim')

function sysCall_init()
    print('========================')
    print('开始创建B型号产品')
    print('========================')

    local Cell = sim.getObject('/FiveCR5A_Cell')
    if Cell == -1 then
        print('[ERROR] /FiveCR5A_Cell not found. Run Step01 first.')
        return
    end

    -- 检查是否已创建
    local existing = sim.getObject('/FiveCR5A_Cell/PartsB')
    if existing ~= -1 then
        print('[INFO] PartsB already exists, skip creation.')
        print('[INFO] To recreate, delete /FiveCR5A_Cell/PartsB first.')
        return
    end

    ------------------------------------------------
    -- 创建 PartsB 容器
    ------------------------------------------------
    local PartsB = sim.createDummy(0.01)
    sim.setObjectAlias(PartsB, 'PartsB')
    sim.setObjectParent(PartsB, Cell, true)
    -- 隐藏容器自身
    pcall(sim.setObjectInt32Param, PartsB, sim.objintparam_visibility_layer, 0)
    print('创建 PartsB 容器完成')

    ------------------------------------------------
    -- 复制分支工具函数
    ------------------------------------------------
    local function copyBranch(sourcePath, newName)
        local source = sim.getObject(sourcePath)
        if source == -1 then
            print('[WARN] 源对象不存在: ' .. sourcePath)
            return nil
        end

        local objs = sim.getObjectsInTree(source, sim.handle_all, 0)
        local copy = sim.copyPasteObjects(objs, 0)

        local root = nil
        for i = 1, #copy do
            if sim.getObjectParent(copy[i]) == -1 then
                root = copy[i]
                break
            end
        end

        if root then
            sim.setObjectAlias(root, newName)
            sim.setObjectParent(root, PartsB, true)
            print('  生成: ' .. newName)
        end

        return root
    end

    ------------------------------------------------
    -- 修改颜色工具函数
    ------------------------------------------------
    local function changeColor(root, color)
        if root == nil then return end
        local objs = sim.getObjectsInTree(root, sim.handle_all, 0)
        for i = 1, #objs do
            local okType, t = pcall(sim.getObjectType, objs[i])
            if okType and t == sim.object_shape_type then
                pcall(sim.setShapeColor, objs[i], nil, sim.colorcomponent_ambient_diffuse, color)
            end
        end
    end

    ------------------------------------------------
    -- 创建B供料物料
    ------------------------------------------------
    print('复制B供料件...')
    local Box_B      = copyBranch('/FiveCR5A_Cell/Parts/Box_Blank',             'Box_Blank_B')
    local PCB_B      = copyBranch('/FiveCR5A_Cell/Parts/PCB_Supply',            'PCB_Supply_B')
    local Module_B   = copyBranch('/FiveCR5A_Cell/Parts/Control_Module_Supply', 'Control_Module_Supply_B')
    local Terminal_B = copyBranch('/FiveCR5A_Cell/Parts/Terminal_Block_Supply', 'Terminal_Block_Supply_B')

    ------------------------------------------------
    -- 创建B装配体和检测体
    ------------------------------------------------
    print('复制B装配体/检测体...')
    local Assembly_B   = copyBranch('/FiveCR5A_Cell/Parts/Assembly_ControlBox_Product',   'Assembly_ControlBox_Product_B')
    local Inspection_B = copyBranch('/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product', 'Inspection_ControlBox_Product_B')

    ------------------------------------------------
    -- B颜色差异化
    ------------------------------------------------
    print('应用B产品颜色...')
    -- B箱体红色系
    changeColor(Box_B,      {0.85, 0.05, 0.05})
    -- B PCB 红橙色
    changeColor(PCB_B,      {1.00, 0.30, 0.10})
    -- B 控制模块深蓝
    changeColor(Module_B,   {0.15, 0.20, 0.55})
    -- B 端子排橙黄
    changeColor(Terminal_B, {0.95, 0.55, 0.10})
    -- B 装配体/检测体红色
    changeColor(Assembly_B,   {0.85, 0.05, 0.05})
    changeColor(Inspection_B, {0.85, 0.05, 0.05})

    ------------------------------------------------
    -- 添加B箱体黑色端盖（区分A型号）
    ------------------------------------------------
    if Box_B ~= -1 then
        local function createSideCover(name, x)
            local cover = sim.createPrimitiveShape(
                sim.primitiveshape_cuboid,
                {0.018, 0.12, 0.10},
                0
            )
            sim.setObjectAlias(cover, name)
            pcall(sim.setShapeColor, cover, nil, sim.colorcomponent_ambient_diffuse, {0.02, 0.02, 0.02})

            local pos = sim.getObjectPosition(Box_B, -1)
            pcall(sim.setObjectPosition, cover, -1, {pos[1] + x, pos[2], pos[3] + 0.05})
            pcall(sim.setObjectParent, cover, Box_B, true)
        end

        createSideCover('B_Left_Black_EndCover',  -0.105)
        createSideCover('B_Right_Black_EndCover',  0.105)
        print('  添加B箱体黑色端盖')
    end

    -- 初始隐藏B产品（由Product Stage Controller控制显示）
    local function hideTree(root)
        if root == -1 then return end
        local objs = sim.getObjectsInTree(root, sim.handle_all, 0)
        for i = 1, #objs do
            pcall(sim.setObjectInt32Param, objs[i], sim.objintparam_visibility_layer, 0)
        end
    end
    hideTree(PartsB)

    print('========================')
    print('B型号产品创建完成')
    print('========================')
    print('[NEXT] Disable this script.')
    print('[INFO] B products located at /FiveCR5A_Cell/PartsB/')
    print('[INFO] Use PRODUCT_A / PRODUCT_B commands to switch.')
end

function sysCall_cleanup()
end
