sim = require('sim')

-- =========================================================
-- Create_Direct_Visible_EndEffectors_R1_R5_working_direction_R4fixed.lua
--
-- ??? CoppeliaSim ??????? R1~R5 ?????
-- ???? URDF / STL / mesh ???????????
--
-- R1/R3/R5??????????
-- R2????????
-- R4???????????????
--
-- ???
-- 1. ?? Dummy??? Create_Direct_Tools
-- 2. ?? Non-threaded child script
-- 3. ??????????
-- 4. ???????????
--
-- ????????????
-- local MOUNT_TO_ROBOTS = false
--
-- ???????????
-- local MOUNT_TO_ROBOTS = true

-- 修正版：保持原来能显示模型的结构；方向为 R1/R3/R5=+90°，R2/R4=180°；并优化 R4T 螺丝刀外形。
-- =========================================================

local MOUNT_TO_ROBOTS = true

-- 修正版：保持原来能显示模型的结构；方向为 R1/R3/R5=+90°，R2/R4=180°；并优化 R4T 螺丝刀外形。
local DELETE_OLD_TOOLS = true

-- ??????????????????????
local mountConfigs = {
    -- 最终方向配置：保持能显示模型的创建结构
    -- R1/R3/R5：+90°
    -- R2/R4：180°
    R1 = {tool='R1T', tip='R1_gripper_tip', localPos={0,0,0}, localOri={0,math.rad(90),0}},
    R2 = {tool='R2T', tip='R2_vacuum_tip',  localPos={0,0,0}, localOri={0,math.rad(180),0}},
    R3 = {tool='R3T', tip='R3_gripper_tip', localPos={0,0,0}, localOri={0,math.rad(90),0}},
    R4 = {tool='R4T', tip='R4_tool_tip',    localPos={0,0,0}, localOri={0,math.rad(180),0}},
    R5 = {tool='R5T', tip='R5_gripper_tip', localPos={0,0,0}, localOri={0,math.rad(90),0}},
}

-- ??????????????????????
local testWorldPos = {
    R1T={-0.90, -1.30, 0.95},
    R2T={-0.45, -1.30, 0.95},
    R3T={ 0.00, -1.30, 0.95},
    R4T={ 0.45, -1.30, 0.95},
    R5T={ 0.90, -1.30, 0.95},
}

local COL_METAL  = {0.62,0.62,0.62}
local COL_DARK   = {0.05,0.05,0.05}
local COL_FINGER = {0.10,0.10,0.10}
local COL_RUBBER = {0.02,0.02,0.02}
local COL_BLUE   = {0.00,0.25,0.55}
local COL_TCP    = {1.00,0.70,0.15}

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

-- 安全查找工具根对象：只接受 alias/name 完全等于 R1T/R2T/... 的对象
-- 避免把 R1T_rear_flange 这种子零件当成根对象，导致模型层级被破坏
local function exactName(h,target)
    for _,n in ipairs(names(h)) do
        if n == target then return true end
    end
    return false
end

local function findExactToolRoot(target)
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

local function match(h,target)
    for _,n in ipairs(names(h)) do
        if n==target or string.find(n,target,1,true) then return true end
    end
    return false
end

local function findInTree(root,target)
    if root==-1 then return -1 end
    local ok,objs=pcall(sim.getObjectsInTree,root,sim.handle_all,0)
    if not ok or not objs then return -1 end
    for _,o in ipairs(objs) do
        if match(o,target) then return o end
    end
    return -1
end

local function findAnywhere(target)
    local h=safeGet('/'..target)
    if h~=-1 then return h end
    local ok,objs=pcall(sim.getObjects,sim.handle_all)
    if ok and objs then
        for _,o in ipairs(objs) do
            if match(o,target) then return o end
        end
    end
    return -1
end

local function removeObjectTree(h)
    if h==-1 then return end
    local ok,objs=pcall(sim.getObjectsInTree,h,sim.handle_all,0)
    if ok and objs then sim.removeObjects(objs) else sim.removeObjects({h}) end
end

local function deleteOld(name)
    local h=findExactToolRoot(name)
    if h~=-1 then
        removeObjectTree(h)
        print('[DELETE OLD TOOL ROOT] '..name)
    end
end

local function setShapeCommon(h,color)
    sim.setShapeColor(h,nil,sim.colorcomponent_ambient_diffuse,color)
    pcall(sim.setObjectInt32Param,h,sim.shapeintparam_static,1)
    pcall(sim.setObjectInt32Param,h,sim.shapeintparam_respondable,0)
    pcall(sim.setObjectInt32Param,h,sim.objintparam_visibility_layer,1)
    return h
end

local function box(parent,name,size,pos,color)
    local h=sim.createPureShape(0,16,size,0.001,nil)
    sim.setObjectAlias(h,name)
    sim.setObjectParent(h,parent,true)
    sim.setObjectPosition(h,parent,pos)
    sim.setObjectOrientation(h,parent,{0,0,0})
    setShapeCommon(h,color)
    return h
end

local function cyl(parent,name,radius,length,pos,ori,color)
    -- cylinder default length axis is local Z
    local h=sim.createPureShape(2,16,{radius*2,radius*2,length},0.001,nil)
    sim.setObjectAlias(h,name)
    sim.setObjectParent(h,parent,true)
    sim.setObjectPosition(h,parent,pos)
    sim.setObjectOrientation(h,parent,ori or {0,0,0})
    setShapeCommon(h,color)
    return h
end

local function sphere(parent,name,radius,pos,color)
    local h=sim.createPureShape(1,16,{radius*2,radius*2,radius*2},0.001,nil)
    sim.setObjectAlias(h,name)
    sim.setObjectParent(h,parent,true)
    sim.setObjectPosition(h,parent,pos)
    sim.setObjectOrientation(h,parent,{0,0,0})
    setShapeCommon(h,color)
    return h
end

local function dummy(parent,name,pos)
    local d=sim.createDummy(0.025)
    sim.setObjectAlias(d,name)
    if parent then sim.setObjectParent(d,parent,true) end
    sim.setObjectPosition(d,parent or -1,pos or {0,0,0})
    sim.setObjectOrientation(d,parent or -1,{0,0,0})
    return d
end

local function findLink6(robotName)
    local r=safeGet('/'..robotName)
    if r==-1 then return -1,-1 end
    local l=findInTree(r,'Link6_visual')
    if l==-1 then l=findInTree(r,'Link6') end
    return r,l
end

local function mountOrPlace(toolRoot,robotName,tipName,tcp)
    if MOUNT_TO_ROBOTS then
        local r,l=findLink6(robotName)
        if r~=-1 and l~=-1 then
            local cfg=mountConfigs[robotName]
            sim.setObjectParent(toolRoot,l,false)
            sim.setObjectPosition(toolRoot,l,cfg.localPos)
            sim.setObjectOrientation(toolRoot,l,cfg.localOri)
            print('[MOUNT OK] '..cfg.tool..' -> '..robotName..'/Link6_visual')
        else
            print('[MOUNT WARN] cannot find '..robotName..' Link6, place in world')
            sim.setObjectParent(toolRoot,-1,true)
            sim.setObjectPosition(toolRoot,-1,testWorldPos[sim.getObjectAlias(toolRoot,0)])
        end
    else
        local name=sim.getObjectAlias(toolRoot,0)
        sim.setObjectParent(toolRoot,-1,true)
        sim.setObjectPosition(toolRoot,-1,testWorldPos[name])
        sim.setObjectOrientation(toolRoot,-1,{0,0,0})
    end

    local tip=dummy(tcp,tipName,{0,0,0})
    return tip
end

local function createGripper(toolName,robotName,tipName,scale,gripMode)
    if DELETE_OLD_TOOLS then deleteOld(toolName) end

    local root=dummy(nil,toolName,{0,0,0})

    -- gripMode:
    -- 'wide'：R1/R3/R5 宽口夹爪；R1 既夹箱体，也夹端子排；
    -- 'small'：备用小夹爪。
    local isWide = (gripMode == 'wide')

    -- 关键修正：
    -- 之前只是把左右 finger_link 拉宽，但前端黑色夹指长度不够，
    -- 视觉上就像“夹爪断开”。这里把夹指做成连续一体式长夹指，
    -- 并增加横向导轨和滑块，让张开时也能看出是一个完整机构。
    local bodyY = isWide and 0.205 or (0.080*scale)
    local railY = isWide and 0.205 or (0.018*scale)
    local baseY = isWide and 0.085 or (0.038*scale)

    cyl(root,toolName..'_rear_flange',0.045*scale,0.035*scale,{0.000,0,0},{0,math.rad(90),0},COL_METAL)
    cyl(root,toolName..'_rear_cylinder',0.035*scale,0.070*scale,{-0.045*scale,0,0},{0,math.rad(90),0},COL_METAL)

    box(root,toolName..'_main_body',{0.120*scale,bodyY,0.045*scale},{-0.115*scale,0,0},COL_METAL)

    -- 静态横向导轨，跨过左右夹爪的运动范围，避免视觉断裂
    box(root,toolName..'_top_rail',{0.165*scale,railY,0.014*scale},{-0.160*scale,0,0.038*scale},COL_METAL)
    box(root,toolName..'_bottom_rail',{0.165*scale,railY,0.014*scale},{-0.160*scale,0,-0.038*scale},COL_METAL)

    if isWide then
        cyl(root,toolName..'_wide_guide_top',0.0060,0.200,{-0.185*scale,0,0.046*scale},{math.rad(90),0,0},COL_DARK)
        cyl(root,toolName..'_wide_guide_bottom',0.0060,0.200,{-0.185*scale,0,-0.046*scale},{math.rad(90),0,0},COL_DARK)
        -- 中间固定背板，说明两侧手指属于同一个夹爪机构
        box(root,toolName..'_wide_back_bridge',{0.040*scale,0.205,0.065*scale},{-0.205*scale,0,0},COL_METAL)
    else
        cyl(root,toolName..'_side_pneumatic_A',0.010*scale,0.045*scale,{-0.090*scale,0.055*scale,0.025*scale},{math.rad(90),0,0},COL_DARK)
        cyl(root,toolName..'_side_pneumatic_B',0.010*scale,0.045*scale,{-0.090*scale,0.055*scale,-0.025*scale},{math.rad(90),0,0},COL_DARK)
    end

    local left=dummy(root,toolName..'_left_finger_link',{0,baseY,0})
    local right=dummy(root,toolName..'_right_finger_link',{0,-baseY,0})

    local function buildFinger(parent,prefix,sgn)
        local y=0

        if isWide then
            -- 宽口一体式夹指：从滑块一直延伸到前端夹持垫，中间不留空隙
            box(parent,prefix..'_slider',{0.055*scale,0.030*scale,0.048*scale},{-0.150*scale,y,0},COL_METAL)
            box(parent,prefix..'_carriage',{0.052*scale,0.030*scale,0.080*scale},{-0.205*scale,y,0},COL_METAL)

            -- 连续上/下长夹指，长度加长，和前端夹持垫重叠
            box(parent,prefix..'_upper_integrated_finger',{0.175,0.022*scale,0.020*scale},{-0.275,y,0.034*scale},COL_FINGER)
            box(parent,prefix..'_lower_integrated_finger',{0.175,0.022*scale,0.020*scale},{-0.275,y,-0.034*scale},COL_FINGER)

            -- 前端竖直夹持块，和上下长夹指重叠连接
            box(parent,prefix..'_front_vertical_jaw',{0.040,0.026*scale,0.095*scale},{-0.350,y,0},COL_FINGER)

            -- 夹持橡胶垫，贴在内侧
            box(parent,prefix..'_inner_rubber_pad',{0.018,0.030*scale,0.075*scale},{-0.368,y,0},COL_RUBBER)

            -- 小螺钉装饰
            cyl(parent,prefix..'_screw_1',0.006*scale,0.024*scale,{-0.205*scale,y+sgn*0.014*scale,0.022*scale},{math.rad(90),0,0},COL_METAL)
            cyl(parent,prefix..'_screw_2',0.006*scale,0.024*scale,{-0.205*scale,y+sgn*0.014*scale,-0.022*scale},{math.rad(90),0,0},COL_METAL)
        else
            box(parent,prefix..'_slider',{0.045*scale,0.026*scale,0.040*scale},{-0.145*scale,y,0},COL_METAL)
            box(parent,prefix..'_adapter_plate',{0.055*scale,0.012*scale,0.060*scale},{-0.175*scale,y,0},COL_METAL)
            box(parent,prefix..'_long_finger_bar',{0.145*scale,0.020*scale,0.018*scale},{-0.255*scale,y,0.028*scale},COL_FINGER)
            box(parent,prefix..'_lower_finger_bar',{0.105*scale,0.020*scale,0.018*scale},{-0.245*scale,y,-0.032*scale},COL_FINGER)
            box(parent,prefix..'_front_hook',{0.030*scale,0.022*scale,0.075*scale},{-0.330*scale,y,-0.002*scale},COL_FINGER)
            box(parent,prefix..'_front_pad_upper',{0.035*scale,0.024*scale,0.020*scale},{-0.350*scale,y,0.040*scale},COL_FINGER)
            box(parent,prefix..'_front_pad_lower',{0.035*scale,0.024*scale,0.020*scale},{-0.350*scale,y,-0.040*scale},COL_FINGER)
            cyl(parent,prefix..'_screw_1',0.006*scale,0.024*scale,{-0.185*scale,y+sgn*0.013*scale,0.018*scale},{math.rad(90),0,0},COL_METAL)
            cyl(parent,prefix..'_screw_2',0.006*scale,0.024*scale,{-0.185*scale,y+sgn*0.013*scale,-0.018*scale},{math.rad(90),0,0},COL_METAL)
        end
    end

    buildFinger(left,toolName..'_left',1)
    buildFinger(right,toolName..'_right',-1)

    local tcpX = isWide and -0.370 or (-0.355*scale)
    local tcp=dummy(root,toolName..'_tool_tcp',{tcpX,0,0})
    sphere(tcp,toolName..'_tcp_marker',0.010*scale,{0,0,0},COL_TCP)

    mountOrPlace(root,robotName,tipName,tcp)
    return root
end

local function createSuction(toolName,robotName,tipName)
    if DELETE_OLD_TOOLS then deleteOld(toolName) end

    local root=dummy(nil,toolName,{0,0,0})

    cyl(root,toolName..'_mount',0.035,0.030,{0,0,0},{0,0,0},COL_METAL)
    cyl(root,toolName..'_stem',0.012,0.070,{0,0,-0.050},{0,0,0},COL_METAL)
    box(root,toolName..'_plate',{0.150,0.090,0.012},{0,0,-0.092},COL_METAL)

    local cupPos={
        {-0.050,-0.030,-0.112},
        { 0.050,-0.030,-0.112},
        {-0.050, 0.030,-0.112},
        { 0.050, 0.030,-0.112},
    }
    for i,p in ipairs(cupPos) do
        cyl(root,toolName..'_cup_'..i,0.016,0.014,p,{0,0,0},COL_RUBBER)
        cyl(root,toolName..'_cup_pipe_'..i,0.004,0.030,{p[1],p[2],-0.100},{0,0,0},COL_DARK)
    end

    local tcp=dummy(root,toolName..'_tool_tcp',{0,0,-0.124})
    sphere(tcp,toolName..'_tcp_marker',0.008,{0,0,0},COL_TCP)

    mountOrPlace(root,robotName,tipName,tcp)
    return root
end

local function createScrewdriver(toolName,robotName,tipName)
    if DELETE_OLD_TOOLS then deleteOld(toolName) end

    local root=dummy(nil,toolName,{0,0,0})

    -- R4 电动螺丝刀修正版：
    -- 保持原脚本“先建模型、再 mountOrPlace”的稳定结构；
    -- 只把外形改得更像电批：短安装座 + 小电批本体 + 细刀杆 + 红色刀尖。
    -- 后续控制脚本仍然旋转 R4T_screw_spin_link。
    cyl(root,toolName..'_mount_plate',0.024,0.020,{0,0,0},{0,0,0},COL_DARK)
    cyl(root,toolName..'_driver_body',0.016,0.060,{0,0,-0.040},{0,0,0},COL_METAL)

    -- 电批握持/电机小方块，增强视觉效果，不参与控制
    box(root,toolName..'_motor_block',{0.030,0.026,0.030},{0,0,-0.038},COL_METAL)

    -- 旋转组，名字不要改，Step02B 控制脚本要用它
    local spin=dummy(root,toolName..'_screw_spin_link',{0,0,-0.080})

    -- 细长刀杆和刀尖
    cyl(spin,toolName..'_bit_shank',0.0035,0.075,{0,0,-0.038},{0,0,0},COL_METAL)
    cyl(spin,toolName..'_bit_tip',0.0020,0.026,{0,0,-0.090},{0,0,0},COL_DARK)

    -- 蓝色偏心标记：旋转时更容易看出刀头在转
    box(spin,toolName..'_bit_rotation_marker',{0.005,0.003,0.045},{0.006,0,-0.038},COL_BLUE)

    -- TCP 放在刀尖末端，名字不要改
    local tcp=dummy(spin,toolName..'_tool_tcp',{0,0,-0.106})
    sphere(tcp,toolName..'_tcp_marker',0.007,{0,0,0},COL_TCP)

    mountOrPlace(root,robotName,tipName,tcp)
    return root
end

function sysCall_init()
    print('===== Create direct visible end-effectors R1-R5 =====')

    createGripper('R1T','R1','R1_gripper_tip',0.88,'wide')
    createSuction('R2T','R2','R2_vacuum_tip')
    createGripper('R3T','R3','R3_gripper_tip',0.88,'wide')
    createScrewdriver('R4T','R4','R4_tool_tip')
    createGripper('R5T','R5','R5_gripper_tip',0.88,'wide')

    print('===== Done. Disable this script after success. =====')
    if MOUNT_TO_ROBOTS then
        print('[INFO] Tools are mounted to robot Link6_visual.')
    else
        print('[INFO] Tools are placed in world row for display test.')
    end
end

function sysCall_cleanup()
end