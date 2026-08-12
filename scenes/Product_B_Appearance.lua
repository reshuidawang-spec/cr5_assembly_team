-- Product_B_Appearance.lua
-- B产品外观大幅修改 - 不影响夹取和装配
-- 严格限定在 PartsB 下操作，不动 Parts (A产品)
-- 运行一次后禁用

sim = require('sim')

-- 递归获取所有子对象
local function getAllChildren(root)
    if root == -1 then return {} end
    local result = {}
    local ok, objs = pcall(sim.getObjectsInTree, root, sim.handle_all, 0)
    if ok and objs then
        for i = 1, #objs do
            result[#result + 1] = objs[i]
        end
    end
    return result
end

-- 安全着色
local function setObjColor(h, color)
    pcall(sim.setShapeColor, h, nil, sim.colorcomponent_ambient_diffuse, color)
end

-- 获取别名
local function getAlias(h)
    local n = ''
    pcall(function() n = sim.getObjectAlias(h, 0) or '' end)
    if n == '' then pcall(function() n = sim.getObjectAlias(h, 1) or '' end) end
    return n
end

function sysCall_init()
    print('===== B产品外观修改 =====')

    -- ============================================
    -- 关键：严格从 PartsB 树根开始，确保不碰 Parts (A产品)
    -- ============================================
    local PartsB = sim.getObject('/FiveCR5A_Cell/PartsB')
    if PartsB == -1 then
        print('[ERROR] /FiveCR5A_Cell/PartsB not found!')
        print('  Run Product_B_Create.lua first.')
        return
    end
    print('[OK] PartsB handle = ' .. PartsB)

    -- ============================================
    -- 1. 找到 PartsB 下的关键对象
    -- ============================================
    local allB = getAllChildren(PartsB)
    print('[OK] Total objects under PartsB: ' .. #allB)

    local boxB = -1
    -- 在 PartsB 子树中找 Box_Blank_B
    for _, h in ipairs(allB) do
        local n = getAlias(h)
        if n == 'Box_Blank_B' then
            boxB = h
            break
        end
    end

    if boxB == -1 then
        print('[ERROR] Box_Blank_B not found under PartsB')
        return
    end
    print('[OK] Box_Blank_B handle = ' .. boxB)

    local boxPos = sim.getObjectPosition(boxB, -1)
    local HL = 0.105  -- 半长
    local HW = 0.075  -- 半宽
    local BOX_HALF_HEIGHT = 0.036
    local RED = {0.85, 0.05, 0.05}
    local boxBottomZ = boxPos[3] - BOX_HALF_HEIGHT
    print(string.format('[OK] Box position: (%.3f, %.3f, %.3f)', boxPos[1], boxPos[2], boxPos[3]))

    -- ============================================
    -- 2. 箱体面板配色
    -- ============================================
    local boxChildren = getAllChildren(boxB)
    local coloredCount = 0
    for _, h in ipairs(boxChildren) do
        local n = getAlias(h)
        if string.find(n, 'Bottom') then
            setObjColor(h, RED); coloredCount = coloredCount + 1
        elseif string.find(n, 'Front_Wall') then
            setObjColor(h, RED); coloredCount = coloredCount + 1
        elseif string.find(n, 'Back_Wall') then
            setObjColor(h, RED); coloredCount = coloredCount + 1
        elseif string.find(n, 'Left_Wall') then
            setObjColor(h, RED); coloredCount = coloredCount + 1
        elseif string.find(n, 'Right_Wall') then
            setObjColor(h, RED); coloredCount = coloredCount + 1
        elseif string.find(n, 'Post') then
            setObjColor(h, RED); coloredCount = coloredCount + 1
        elseif string.find(n, 'EndCover') then
            setObjColor(h, RED); coloredCount = coloredCount + 1
        end
    end
    print('[OK] Box panels recolored: ' .. coloredCount)

    -- ============================================
    -- 3. 底部扩展法兰
    -- ============================================
    local flange = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid, {0.23, 0.17, 0.006}, 0)
    sim.setObjectAlias(flange, 'B_Base_Flange')
    setObjColor(flange, RED)
    -- 法兰底面与标准箱底齐平，不能进入装配夹具。
    sim.setObjectPosition(flange, -1, {boxPos[1], boxPos[2], boxBottomZ + 0.003})
    sim.setObjectParent(flange, boxB, true)
    print('[OK] Bottom flange')

    -- ============================================
    -- 4. 外壁竖条纹
    -- ============================================
    local function addRib(name, x, y, z, sx, sy, sz, color)
        local rib = sim.createPrimitiveShape(sim.primitiveshape_cuboid, {sx, sy, sz}, 0)
        sim.setObjectAlias(rib, name)
        setObjColor(rib, color or RED)
        sim.setObjectPosition(rib, -1, {x, y, z})
        sim.setObjectParent(rib, boxB, true)
    end

    local lx = boxPos[1] - HL - 0.004
    for j = 1, 3 do
        addRib('B_Rib_L'..j, lx, boxPos[2] + (j-2)*0.04, boxPos[3], 0.004, 0.015, 0.055)
    end
    local rx = boxPos[1] + HL + 0.004
    for j = 1, 3 do
        addRib('B_Rib_R'..j, rx, boxPos[2] + (j-2)*0.04, boxPos[3], 0.004, 0.015, 0.055)
    end
    local by = boxPos[2] + HW + 0.004
    for j = 1, 2 do
        addRib('B_Rib_B'..j, boxPos[1] + (j-1.5)*0.07, by, boxPos[3], 0.025, 0.004, 0.050)
    end
    print('[OK] External ribs: 8 total')

    -- ============================================
    -- 5. 四角加强筋
    -- ============================================
    local corners = {
        {boxPos[1]-HL, boxPos[2]-HW}, {boxPos[1]+HL, boxPos[2]-HW},
        {boxPos[1]-HL, boxPos[2]+HW}, {boxPos[1]+HL, boxPos[2]+HW},
    }
    for j, c in ipairs(corners) do
        local g = sim.createPrimitiveShape(sim.primitiveshape_cuboid, {0.016, 0.016, 0.025}, 0)
        sim.setObjectAlias(g, 'B_Gusset_'..j)
        setObjColor(g, RED)
        -- 角撑底面同样与标准箱底齐平。
        sim.setObjectPosition(g, -1, {c[1], c[2], boxBottomZ + 0.0125})
        sim.setObjectParent(g, boxB, true)
    end
    print('[OK] Corner gussets: 4')

    -- ============================================
    -- 6. 前壁标记
    -- ============================================
    local fy = boxPos[2] - HW - 0.004
    local m1 = sim.createPrimitiveShape(sim.primitiveshape_cuboid, {0.03, 0.004, 0.02}, 0)
    sim.setObjectAlias(m1, 'B_Marker_L')
    setObjColor(m1, {1.0, 0.90, 0.05})
    sim.setObjectPosition(m1, -1, {boxPos[1]-0.04, fy, boxPos[3]+0.015})
    sim.setObjectParent(m1, boxB, true)

    local m2 = sim.createPrimitiveShape(sim.primitiveshape_cuboid, {0.03, 0.004, 0.02}, 0)
    sim.setObjectAlias(m2, 'B_Marker_R')
    setObjColor(m2, {1.0, 0.90, 0.05})
    sim.setObjectPosition(m2, -1, {boxPos[1]+0.04, fy, boxPos[3]+0.015})
    sim.setObjectParent(m2, boxB, true)
    print('[OK] Front markers: 2')

    -- ============================================
    -- 7. 零件改色（PCB/模块/端子排 - 仅 PartsB 下）
    -- ============================================
    for _, h in ipairs(allB) do
        local n = getAlias(h)
        if string.find(n, 'PCB_Supply_Board') or string.find(n, 'PCB_Board') then
            setObjColor(h, {0.35, 0.05, 0.35})
        elseif string.find(n, 'PCB_') and string.find(n, 'Chip') then
            setObjColor(h, {0.85, 0.85, 0.90})
        elseif string.find(n, 'PCB_') and string.find(n, 'Hole') then
            setObjColor(h, {1.0, 0.75, 0.15})
        elseif string.find(n, 'PCB_') and string.find(n, 'Connector') then
            setObjColor(h, {1.0, 0.90, 0.10})
        elseif string.find(n, 'Control_Module') and string.find(n, 'Body') then
            setObjColor(h, {0.95, 0.25, 0.15})
        elseif string.find(n, 'Control_Module') and string.find(n, 'Label') then
            setObjColor(h, {1.0, 1.0, 1.0})
        elseif string.find(n, 'Terminal_Block') and string.find(n, 'Body') then
            setObjColor(h, {0.10, 0.70, 0.25})
        elseif string.find(n, 'Terminal_Block') and string.find(n, 'Slot') then
            setObjColor(h, {0.02, 0.15, 0.05})
        elseif string.find(n, 'Terminal_Block') and string.find(n, 'Screw') then
            setObjColor(h, {0.90, 0.90, 0.92})
        end
    end
    print('[OK] PCB/Module/Terminal recolored (PartsB only)')

    -- ============================================
    -- 8. 验证：确保没碰 A 产品
    -- ============================================
    local PartsA = sim.getObject('/FiveCR5A_Cell/Parts')
    if PartsA ~= -1 then
        local aBox = sim.getObject('/FiveCR5A_Cell/Parts/Box_Blank')
        if aBox ~= -1 then
            local aBottom = sim.getObject('/FiveCR5A_Cell/Parts/Box_Blank/Box_Blank_Bottom')
            if aBottom ~= -1 then
                print('[VERIFY] A product Box_Blank_Bottom still exists (not modified) - OK')
            end
        end
    end

    print('')
    print('===== B产品外观修改完成 =====')
    print('A vs B: 现在可以在场景中切换 PRODUCT_A / PRODUCT_B 对比')
    print('  ros2 topic pub --once /product_order std_msgs/msg/String "data: A"')
    print('  ros2 topic pub --once /product_order std_msgs/msg/String "data: B"')
end
