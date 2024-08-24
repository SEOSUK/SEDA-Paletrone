from .utils.widgets import MicrostrainWidget, MicrostrainPlugin
from .utils.subscribers import RTKMonitor

_WIDGET_NAME = 'RTKStatus'

class RTKStatusWidget(MicrostrainWidget):

  def __init__(self, node):
    # Initialize the parent class
    super(RTKStatusWidget, self).__init__(node, _WIDGET_NAME)

  def _configure(self):
    # Set up the subscriber status monitors
    self._rtk_status_monitor = RTKMonitor(self._node, self._node_name, "rtk/status")

    # Hide the warning label
    self.rtk_not_available_label.hide()

  def run(self):
    # If the device is connected and not a GQ7, display a warning
    if self._device_report_monitor.connected and not self._device_report_monitor.is_gq7:
      self.rtk_widget.hide()
      self.rtk_not_available_label.setText('RTK status only available for GQ7 devices. Not available for device %s' % self._device_report_monitor.model_name_string)
      self.rtk_not_available_label.show()
      return
    else:
      self.rtk_not_available_label.hide()
      self.rtk_widget.show()

    # Update device specific data
    self._update_rtk_data()
    
  def _update_rtk_data(self):
    # Epoch Status flags
    self.rtk_corrections_received_gps_label.setText(self._rtk_status_monitor.gps_received_string)
    self.rtk_corrections_received_glonass_label.setText(self._rtk_status_monitor.glonass_received_string)
    self.rtk_corrections_received_galileo_label.setText(self._rtk_status_monitor.galileo_received_string)
    self.rtk_corrections_received_beidou_label.setText(self._rtk_status_monitor.beidou_received_string)

    # Icon and label
    self.rtk_led_icon_label.setText(self._rtk_status_monitor.rtk_led_string)
    self.rtk_led_status_label.setText(self._rtk_status_monitor.controller_status_string)

    # Other Status Flags
    self.rtk_status_flags_mode_label.setText(self._rtk_status_monitor.controller_state_string)
    self.rtk_status_flags_controller_status_label.setText(self._rtk_status_monitor.controller_status_string)
    self.rtk_status_flags_device_state_label.setText(self._rtk_status_monitor.platform_state_string)
    self.rtk_status_flags_connection_status_label.setText(self._rtk_status_monitor.platform_status_string)
    self.rtk_status_flags_reset_reason_label.setText(self._rtk_status_monitor.reset_reason_string)
    self.rtk_status_flags_signal_quality_label.setText(self._rtk_status_monitor.signal_quality_string)

class RTKStatusPlugin(MicrostrainPlugin):

  def __init__(self, context):
    # Initialize the parent class
    super(RTKStatusPlugin, self).__init__(context, _WIDGET_NAME, RTKStatusWidget)
