# This supports only polled analog and digital input on a single Labjack U3-HV board at this time.
# There is currently no on-the-fly reading of parameters

import rclpy
import rclpy.node
from u3 import U3 as lj
import u3
from functools import partial
from ssp_interfaces.msg import AnalogInput, DigitalInput

class LabjackNode(rclpy.node.Node):
    def __init__(self, name):
        super().__init__(name)
        self.SINGLE_END_REF_CHAN    = 31
        self.MAX_BIN_VAL            = 65536
        self.SPAN_LV_SE             = 2.44
        self.SPAN_LV_DIFF           = 4.88
        self.SPAN_HV                = 20.6
        self.MIN_CHAN               = 0
        self.MAX_CHAN               = 20
        self.MIN_DIGITAL_CHAN       = 4
        self.MAX_DIGITAL_CHAN       = self.MAX_CHAN
        self.MIN_ANALOG_CHAN        = self.MIN_CHAN
        self.MAX_ANALOG_CHAN        = 16
        self.MIN_HV_ANALOG_CHAN     = self.MIN_CHAN
        self.MAX_HV_ANALOG_CHAN     = 4
        self.timer_list             = []
        self.publisher_list         = []
        self.declare_params()
        self.params = self.fetch_params()
        self.dev = lj()
        self.config_labjack()
        self.create_timed_pubs()
        self.frame_id = ""

    def analog_timer_cb(self, channel):
        msg = AnalogInput()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        neg_channel = self.params['channels'][channel]['analog']['negative_channel']
        offset = self.params['channels'][channel]['analog']['offset']
        gain = self.params['channels'][channel]['analog']['gain']
        try:
            msg.raw_value = self.dev.getFeedback(u3.AIN(channel, neg_channel))[0]
        except Exception as e:
            self.get_logger().error("Attempt to read analog value on channel {0} failed, error {1}".format(channel, str(e)))
        is_lv = True
        is_se = False
        if (channel < self.MAX_HV_ANALOG_CHAN): is_lv = False
        if (neg_channel == self.SINGLE_END_REF_CHAN): is_se = True
        msg.scaled_voltage = self.dev.binaryToCalibratedAnalogVoltage(
            msg.raw_value, isLowVoltage=is_lv, isSingleEnded=is_se, 
            isSpecialSetting=False, channelNumber=channel)
        msg.scaled_value = (msg.scaled_voltage + offset) * gain
        try:
            self.publisher_list[channel].publish(msg)
        except Exception as e:
            self.get_logger().error("Attempt to publish analog value on channel {0} failed, error {1}".format(channel, str(e)))

    def digital_timer_cb(self, channel):
        msg = DigitalInput()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        try:
            msg.state = bool(self.dev.getDIState(channel))
        except Exception as e:
            self.get_logger().error("Attempt to read digital value on channel {0} failed, error {1}".format(channel, str(e)))
        try:
            self.publisher_list[channel].publish(msg)
        except Exception as e:
            self.get_logger().error("Attempt to publish digital value on channel {0} failed, error {1}".format(channel, str(e)))

    def declare_params(self):
        self.declare_parameter('frame_id', '')
        for channel in range(self.MIN_CHAN, self.MAX_CHAN):
            self.declare_parameter('channel.' + str(channel) + '.active', False)        # Is the channel active?
            self.declare_parameter('channel.' + str(channel) + '.period', 1.0)          # Period for sampling the channel, in seconds
        for channel in range(self.MIN_HV_ANALOG_CHAN, self.MAX_HV_ANALOG_CHAN):
            self.declare_parameter('channel.' + str(channel) + '.is_analog', True)     # Is the channel analog?
        for channel in range(self.MIN_DIGITAL_CHAN, self.MAX_DIGITAL_CHAN):
            self.declare_parameter('channel.' + str(channel) + '.is_analog', False)     # Is the channel analog?
        for channel in range(self.MIN_ANALOG_CHAN, self.MAX_ANALOG_CHAN):
            self.declare_parameter('channel.' + str(channel) + '.analog.negative_channel', self.SINGLE_END_REF_CHAN)  # Defaults to single ended
            self.declare_parameter('channel.' + str(channel) + '.analog.offset', 0.0)   # offset to convert volts to engineering units
            self.declare_parameter('channel.' + str(channel) + '.analog.gain', 1.0)     # gain to convert volts to engineering units
   
    def fetch_params(self):
        results = {'channels' : []}
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        # First, fix some parameter values to values required by the hardware
        hv_fixed_params = []
        gpio_fixed_params = []
        for channel in range(self.MIN_HV_ANALOG_CHAN, self.MAX_HV_ANALOG_CHAN):
            # Must be analog
            hv_fixed_params.append(rclpy.parameter.Parameter(
                'channel.' + str(channel) + '.is_analog', 
                rclpy.Parameter.Type.BOOL, 
                True))
            # Must be single ended
            hv_fixed_params.append(rclpy.parameter.Parameter(
                'channel.' + str(channel) + '.analog.negative_channel', 
                rclpy.Parameter.Type.INTEGER, 
                31))
        for channel in range((self.MAX_ANALOG_CHAN + 1), self.MAX_DIGITAL_CHAN):
            # Must be digital
            gpio_fixed_params.append(rclpy.parameter.Parameter(
                'channel.' + str(channel) + '.is_analog', 
                rclpy.Parameter.Type.BOOL, 
                False))
        self.set_parameters(hv_fixed_params)
        self.set_parameters(gpio_fixed_params)

        # Second, fetch parameters common to all channels
        for channel in range(self.MIN_CHAN, self.MAX_CHAN):
            results['channels'].insert(channel, {})
            results['channels'][channel]['active'] = self.get_parameter(
                'channel.' + str(channel) + '.active').get_parameter_value().bool_value
            results['channels'][channel]['period'] = self.get_parameter(
                'channel.' + str(channel) + '.period').get_parameter_value().double_value
            results['channels'][channel]['is_analog'] = self.get_parameter(
                'channel.' + str(channel) + '.is_analog').get_parameter_value().bool_value

        # Third, fetch analog specific parameters
        for channel in range(self.MIN_ANALOG_CHAN, self.MAX_ANALOG_CHAN):
            results['channels'][channel]['analog'] = {}
            results['channels'][channel]['analog']['negative_channel'] = self.get_parameter(
                'channel.' + str(channel) + '.analog.negative_channel').get_parameter_value().integer_value
            results['channels'][channel]['analog']['offset'] = self.get_parameter(
                'channel.' + str(channel) + '.analog.offset').get_parameter_value().double_value
            results['channels'][channel]['analog']['gain'] = self.get_parameter(
                'channel.' + str(channel) + '.analog.gain').get_parameter_value().double_value

        self.get_logger().info('Labjack parameters: ' + str(results))

        # Finally, return what we've assembled
        return results

    def config_labjack(self):
        eio_analog  = 0x00
        fio_analog  = 0x0f
        for channel in range(self.MIN_CHAN, self.MAX_CHAN):
            if self.params['channels'][channel]['active']:
                if self.params['channels'][channel]['is_analog']:
                    if channel < 8:
                        fio_analog = fio_analog | (1 << channel)
                    elif channel < 16:
                        eio_analog = eio_analog | (1 << (channel - 8))
                else:
                    self.dev.getFeedback(u3.BitDirWrite(IONumber = channel, Direction = 0))
        self.get_logger().info("Setting FIOAnalog to {0} and EIOAnalog to {1}".format(fio_analog, eio_analog))
        self.dev.configIO(FIOAnalog=fio_analog, EIOAnalog=eio_analog)

    def create_timed_pubs(self):
        for chan in range(self.MIN_CHAN, self.MAX_CHAN):
            if self.params['channels'][chan]['active']:
                period = self.params['channels'][chan]['period']
                if self.params['channels'][chan]['is_analog']:
                    self.timer_list.append(self.create_timer(period, partial(self.analog_timer_cb, channel=chan)))
                    self.publisher_list.append(self.create_publisher(AnalogInput, '~/analog/ch' + str(chan), 10))
                else:
                    self.timer_list.append(self.create_timer(period, partial(self.digital_timer_cb, channel=chan)))
                    self.publisher_list.append(self.create_publisher(DigitalInput, '~/digital/ch' + str(chan), 10))
            else:
                self.timer_list.append(None)
                self.publisher_list.append(None)


def main(args=None):
    rclpy.init(args=args)
    node = LabjackNode('labjack_uc3_hv')
    rclpy.spin(node)

if __name__ == '__main__':
    main()
