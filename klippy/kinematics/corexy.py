  # Code for handling the kinematics of corexy robots
    #
    # Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
    #
    # This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper

# 配置日志记录
logging.basicConfig(level=logging.DEBUG)

class CoreXYKinematics:
    def __init__(self, toolhead, config):
        logging.debug("Initializing CoreXYKinematics")
        
        # 初始化轴轨道配置
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzab']  # 包含 'a' 和 'b' 轴的初始化
        
        # 绑定对应轴的步进器和限位开关
        for s in self.rails[1].get_steppers():
            self.rails[0].get_endstops()[0][0].add_stepper(s)
        for s in self.rails[0].get_steppers():
            self.rails[1].get_endstops()[0][0].add_stepper(s)
        
        # 初始化轴的迭代解算器配置
        self.rails[0].setup_itersolve('corexy_stepper_alloc', b'+')
        self.rails[1].setup_itersolve('corexy_stepper_alloc', b'-')
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')
        self.rails[3].setup_itersolve('cartesian_stepper_alloc', b'a')
        self.rails[4].setup_itersolve('cartesian_stepper_alloc', b'b')
        
        # 将每个斜块的轨迹查询队列设置为打印头封装的轨迹查询队列
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        
        # 注册电机关闭时的事件处理函数
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        
        # 设置速度和加速度限制，如有必要，可添加 A 和 B 轴的额外配置
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_velocity_limits = {}
        self.max_acceleration_limits = {}
        for axis in 'xyzab':
            self.max_velocity_limits[axis] = config.getfloat(
                f'max_{axis}_velocity', max_velocity, above=0., maxval=max_velocity)
            self.max_acceleration_limits[axis] = config.getfloat(
                f'max_{axis}_accel', max_accel, above=0., maxval=max_accel)
        
        # 设置轴限制和最小最大位置
        self.limits = [(1.0, -1.0)] * 5
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)

    def get_steppers(self):
        steppers = [s for rail in self.rails for s in rail.get_steppers()]
        logging.debug(f"Steppers: {steppers}")
        return steppers

    def calc_position(self, stepper_positions):
        logging.debug(f"Calculating position with stepper positions: {stepper_positions}")
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        # 计算包含 'a' 和 'b' 轴的位置
        calculated_position = [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2], pos[3], pos[4]]
        logging.debug(f"Calculated position: {calculated_position}")
        return calculated_position

    def set_position(self, newpos, homing_axes):
        logging.debug(f"Setting position to: {newpos} for homing axes: {homing_axes}")
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()

    def note_z_not_homed(self):
        logging.debug("Noting Z axis not homed")
        self.limits[2] = (1.0, -1.0)

    def home(self, homing_state):
        logging.debug(f"Homing with state: {homing_state}")
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            rail = self.rails[axis]
            # Determine movement
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None, None, None]  # include A and B axis
            homepos[axis] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            homing_state.home_rails([rail], forcepos, homepos)

    def _motor_off(self, print_time):
        logging.debug(f"Motor off at print time: {print_time}")
        # 关闭电机时重置限制
        self.limits = [(1.0, -1.0)] * 5  # include A and B axis

    def _check_endstops(self, move):
        logging.debug(f"Checking endstops for move: {move}")
        # 检查轴限制并引发错误（如果需要）
        end_pos = move.end_pos
        for i, limit in enumerate(self.limits):  # checking all 5 axes
            if move.axes_d[i] and (end_pos[i] < limit[0] or end_pos[i] > limit[1]):
                if limit[0] > limit[1]:
                    raise move.move_error(f"Must home axis {'xyzab'[i]} first")
                raise move.move_error(f"Axis {'xyzab'[i]} move out of range")

    def check_move(self, move):
        logging.debug(f"Checking move: {move}")
        # 检查移动限制并按轴更新速度和加速度
        for axis in 'xyzab':  # checking all 5 axes
            i = 'xyzab'.index(axis)
            pos = move.end_pos[i]
            if pos < self.limits[i][0] or pos > self.limits[i][1]:
                self._check_endstops(move)
            if move.axes_d[i]:
                # Update move speed and acceleration for the axis
                ratio = move.move_d / abs(move.axes_d[i])
                move.limit_speed(
                    self.max_velocity_limits[axis] * ratio,
                    self.max_acceleration_limits[axis] * ratio)

    def get_status(self, eventtime):
        logging.debug(f"Getting status at event time: {eventtime}")
        # 返回轴的状态信息
        axes = [a for a, (l, h) in zip("xyzab", self.limits) if l <= h]
        status = {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }
        logging.debug(f"Status: {status}")
        return status

def load_kinematics(toolhead, config):
    logging.debug("Loading CoreXYKinematics")
    return CoreXYKinematics(toolhead, config)
