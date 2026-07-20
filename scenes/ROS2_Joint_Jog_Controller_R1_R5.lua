sim = require('sim')

-- =========================================================
-- ROS2_Joint_Jog_Controller_R1_R5.lua
--
-- 作用：
-- 通过 ROS2 控制 R1~R5 机械臂的关节角。
--
-- 订阅 topic：
--   /compact_cell/joint_cmd    std_msgs/msg/String
--
-- 发布状态：
--   /compact_cell/joint_status std_msgs/msg/String
--
-- 支持命令格式：
--
-- 1）单关节增量运动，单位：度
--   R1 J1 +10
--   R1 J2 -5
--   R3 J6 +20
--
-- 2）单关节设置绝对角度，单位：度
--   R1 J1 =30
--   R2 J4 =-45
--
-- 3）六关节一次性设置绝对角度，单位：度
--   R1 SET 0 20 -30 0 45 0
--   R3 SET 10 0 -20 0 30 0
--
-- 4）回零
--   R1 HOME
--   R2 HOME
--   ALL HOME
--
-- 注意：
-- 这是关节点动测试脚本，不是路径规划。
-- 它只负责让关节转动，用来验证：
--   1. CoppeliaSim 能收到 ROS2 指令；
--   2. 能找到各机械臂关节；
--   3. 关节能被 setJointTargetPosition / setJointPosition 控制。
-- =========================================================

local simROS2 = nil
local ros_ok = false
local sub_cmd = nil
local pub_status = nil

local robots = {'R1','R2','R3','R4','R5'}
local jointHandles = {}
local homeDeg = {0,0,0,0,0,0}

-- 控制参数
local USE_TARGET_POSITION_FIRST = true
local JOINT_MOVE_SPEED = 0.5 -- rad/s，仅用于后续扩展；当前直接设目标

local function log(msg)
    print('[JOINT CTRL] '..msg)
    if ros_ok and pub_status then
        pcall(simROS2.publish,pub_status,{data=msg})
    end
end

local function trim(s)
    if not s then return '' end
    s = string.gsub(s,'^%s+','')
    s = string.gsub(s,'%s+$','')
    return s
end

local function splitWords(s)
    local t = {}
    for w in string.gmatch(s,'%S+') do
        table.insert(t,w)
    end
    return t
end

local function deg2rad(d)
    return tonumber(d) * math.pi / 180.0
end

local function rad2deg(r)
    return r * 180.0 / math.pi
end

local function safeGet(path)
    local ok,h = pcall(sim.getObject,path)
    if ok then return h end
    return -1
end

local function getAliasList(h)
    local t = {}
    local ok,a0 = pcall(sim.getObjectAlias,h,0)
    if ok and a0 then table.insert(t,a0) end
    local ok2,a1 = pcall(sim.getObjectAlias,h,1)
    if ok2 and a1 then table.insert(t,a1) end
    local ok3,n = pcall(sim.getObjectName,h)
    if ok3 and n then table.insert(t,n) end
    return t
end

local function containsIgnoreCase(s,pat)
    if not s or not pat then return false end
    return string.find(string.lower(s),string.lower(pat),1,true) ~= nil
end

local function findRobotRoot(robot)
    local h = safeGet('/'..robot)
    if h ~= -1 then return h end

    local ok,objs = pcall(sim.getObjects,sim.handle_all)
    if ok and objs then
        for _,o in ipairs(objs) do
            for _,name in ipairs(getAliasList(o)) do
                if name == robot then return o end
            end
        end
    end
    return -1
end

local function collectJointsInTree(root)
    local joints = {}
    if root == -1 then return joints end

    local ok,objs = pcall(sim.getObjectsInTree,root,sim.object_joint_type,0)
    if ok and objs then
        for _,j in ipairs(objs) do
            table.insert(joints,j)
        end
    end
    return joints
end

local function jointScoreForIndex(j,index)
    local score = 0
    local idxStr = tostring(index)

    for _,name in ipairs(getAliasList(j)) do
        local lname = string.lower(name)

        -- 优先匹配 joint1 / joint_1 / j1 / axis1 这类名字
        if lname == 'joint'..idxStr then score = score + 100 end
        if lname == 'joint_'..idxStr then score = score + 100 end
        if lname == 'j'..idxStr then score = score + 100 end
        if lname == 'axis'..idxStr then score = score + 80 end

        if containsIgnoreCase(lname,'joint'..idxStr) then score = score + 40 end
        if containsIgnoreCase(lname,'joint_'..idxStr) then score = score + 40 end
        if containsIgnoreCase(lname,'j'..idxStr) then score = score + 20 end

        -- 避免 joint10 被 joint1 错匹配，轻微惩罚过长数字
        if containsIgnoreCase(lname,'joint'..idxStr..'0') then score = score - 50 end
    end

    return score
end

local function sortJointsFallback(joints)
    table.sort(joints,function(a,b)
        local na = getAliasList(a)[1] or ''
        local nb = getAliasList(b)[1] or ''
        return na < nb
    end)
end

local function findSixJoints(robot)
    local root = findRobotRoot(robot)
    if root == -1 then
        log('ERROR: robot root not found: '..robot)
        return {}
    end

    local joints = collectJointsInTree(root)
    if #joints < 6 then
        log('ERROR: '..robot..' joints found only '..#joints)
        return joints
    end

    local result = {}
    local used = {}

    for i=1,6 do
        local best = -1
        local bestScore = -999

        for _,j in ipairs(joints) do
            if not used[j] then
                local s = jointScoreForIndex(j,i)
                if s > bestScore then
                    bestScore = s
                    best = j
                end
            end
        end

        if best ~= -1 and bestScore > 0 then
            result[i] = best
            used[best] = true
        end
    end

    -- 如果命名识别不到，则退回按名字排序取前 6 个
    local count = 0
    for i=1,6 do if result[i] then count = count + 1 end end

    if count < 6 then
        sortJointsFallback(joints)
        result = {}
        for i=1,6 do
            result[i] = joints[i]
        end
        log('WARN: '..robot..' joint names not clear, fallback to sorted first 6 joints.')
    end

    for i=1,6 do
        if result[i] then
            local name = getAliasList(result[i])[1] or tostring(result[i])
            log(string.format('%s J%d -> %s',robot,i,name))
        end
    end

    return result
end

local function cacheAllJoints()
    jointHandles = {}
    for _,r in ipairs(robots) do
        jointHandles[r] = findSixJoints(r)
    end
end

local function setJointRad(j,rad)
    if j == nil or j == -1 then return false end

    if USE_TARGET_POSITION_FIRST then
        local ok = pcall(sim.setJointTargetPosition,j,rad)
        if ok then return true end
    end

    local ok2 = pcall(sim.setJointPosition,j,rad)
    return ok2
end

local function getJointRad(j)
    local ok,pos = pcall(sim.getJointPosition,j)
    if ok and pos then return pos end
    return 0
end

local function setOneJoint(robot,jointIndex,valueDeg,absolute)
    if not jointHandles[robot] or not jointHandles[robot][jointIndex] then
        log('ERROR: joint not found: '..robot..' J'..tostring(jointIndex))
        return
    end

    local j = jointHandles[robot][jointIndex]
    local current = getJointRad(j)
    local target

    if absolute then
        target = deg2rad(valueDeg)
    else
        target = current + deg2rad(valueDeg)
    end

    local ok = setJointRad(j,target)
    if ok then
        log(string.format('OK: %s J%d %.2f deg -> %.2f deg',
            robot,jointIndex,rad2deg(current),rad2deg(target)))
    else
        log(string.format('ERROR: failed set %s J%d',robot,jointIndex))
    end
end

local function setRobotPose(robot,values)
    if not jointHandles[robot] then
        log('ERROR: robot not cached: '..robot)
        return
    end

    for i=1,6 do
        if not values[i] then
            log('ERROR: SET needs 6 values.')
            return
        end
    end

    for i=1,6 do
        setJointRad(jointHandles[robot][i],deg2rad(values[i]))
    end

    log(string.format('OK: %s SET [%.1f %.1f %.1f %.1f %.1f %.1f] deg',
        robot,values[1],values[2],values[3],values[4],values[5],values[6]))
end

local function homeRobot(robot)
    if robot == 'ALL' then
        for _,r in ipairs(robots) do
            setRobotPose(r,homeDeg)
        end
        log('OK: ALL HOME')
    else
        setRobotPose(robot,homeDeg)
        log('OK: '..robot..' HOME')
    end
end

local function handleJointCmd(cmd)
    cmd = trim(cmd)
    if cmd == '' then return end

    local w = splitWords(cmd)
    if #w < 2 then
        log('ERROR: bad command: '..cmd)
        return
    end

    local robot = string.upper(w[1])

    -- ALL HOME
    if robot == 'ALL' and string.upper(w[2]) == 'HOME' then
        homeRobot('ALL')
        return
    end

    -- R1 HOME
    if string.upper(w[2]) == 'HOME' then
        homeRobot(robot)
        return
    end

    -- R1 SET a b c d e f
    if string.upper(w[2]) == 'SET' then
        local vals = {}
        for i=1,6 do
            vals[i] = tonumber(w[i+2])
        end
        setRobotPose(robot,vals)
        return
    end

    -- R1 J1 +10 / R1 J1 -10 / R1 J1 =30
    local jointToken = string.upper(w[2])
    local jointIndex = tonumber(string.match(jointToken,'J(%d+)'))

    if not jointIndex or jointIndex < 1 or jointIndex > 6 then
        log('ERROR: bad joint index: '..tostring(w[2]))
        return
    end

    local valToken = w[3]
    if not valToken then
        log('ERROR: missing value.')
        return
    end

    local absolute = false
    local valueDeg = nil

    if string.sub(valToken,1,1) == '=' then
        absolute = true
        valueDeg = tonumber(string.sub(valToken,2))
    else
        absolute = false
        valueDeg = tonumber(valToken)
    end

    if not valueDeg then
        log('ERROR: bad value: '..tostring(valToken))
        return
    end

    setOneJoint(robot,jointIndex,valueDeg,absolute)
end

function jointCmd_cb(msg)
    handleJointCmd(msg.data)
end

function sysCall_init()
    print('===== ROS2 Joint Jog Controller R1~R5 =====')

    cacheAllJoints()

    local ok,modOrErr = pcall(require,'simROS2')
    if not ok then
        print('[JOINT CTRL] simROS2 not available. You can still use internal signal joint_cmd.')
        ros_ok = false
        return
    end

    simROS2 = modOrErr
    ros_ok = true

    local okPub,pubOrErr = pcall(simROS2.createPublisher,'/compact_cell/joint_status','std_msgs/msg/String')
    if okPub and pubOrErr then
        pub_status = pubOrErr
        log('PUB_OK: /compact_cell/joint_status')
    else
        log('PUB_FAILED: /compact_cell/joint_status')
    end

    local okSub,subOrErr = pcall(simROS2.createSubscription,'/compact_cell/joint_cmd','std_msgs/msg/String','jointCmd_cb')
    if okSub and subOrErr then
        sub_cmd = subOrErr
        log('SUB_OK: /compact_cell/joint_cmd')
    else
        log('SUB_FAILED: /compact_cell/joint_cmd err='..tostring(subOrErr))
    end

    log('JOINT_CTRL_READY')
end

function sysCall_actuation()
    -- 也支持 CoppeliaSim 内部 signal 测试：
    -- sim.setStringSignal('joint_cmd','R1 J1 +10')
    local cmd = sim.getStringSignal('joint_cmd')
    if cmd and cmd ~= '' then
        handleJointCmd(cmd)
        sim.clearStringSignal('joint_cmd')
    end
end

function sysCall_cleanup()
    if ros_ok and simROS2 then
        if sub_cmd then pcall(simROS2.shutdownSubscription,sub_cmd) end
        if pub_status then pcall(simROS2.shutdownPublisher,pub_status) end
    end
end
