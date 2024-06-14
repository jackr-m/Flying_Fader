searchState.loadedDescShard("embassy_usb_driver", 0, "embassy-usb-driver\nEither the packet to be written is too long to fit in the …\nBulk endpoint. Used for large amounts of best-effort …\nUSB bus trait.\nType for bus control for this driver.\nControl endpoint. Used for device management. Only the …\nUSB control pipe trait.\nType of the control pipe for this driver.\nDirection of USB traffic. Note that in the USB standard …\nThe endpoint is disabled.\nMain USB driver trait.\nEndpoint trait, common for OUT and IN.\nType-safe endpoint address.\nAllocating an endpoint failed.\nErrors returned by <code>EndpointIn::write</code> and <code>EndpointOut::read</code>\nIN Endpoint trait.\nType of the IN endpoints for this driver.\nInformation for an endpoint.\nOUT Endpoint trait.\nType of the OUT endpoints for this driver.\nUSB endpoint transfer type. The values of this enum can be …\nEvent returned by <code>Bus::poll</code>.\nDevice to host (IN)\nInterrupt endpoint. Used for small amounts of …\nIsochronous endpoint. Used for time-critical unreliable …\nHost to device (OUT)\nThe USB power has been detected.\nThe USB power has been removed. Not supported by all …\nThe USB reset condition has been detected.\nA USB resume request has been detected after being …\nA USB suspend request has been detected or, in the case of …\nOperation is unsupported by the driver.\nAccept a control request.\nAccept SET_ADDRESS control and change bus address.\nEndpoint’s address.\nAllocates an IN endpoint.\nAllocates an OUT endpoint.\nSend a DATA IN packet with <code>data</code> in response to a control …\nRead a DATA OUT packet into <code>buf</code> in response to a control …\nGets the direction part of the address.\nDisable and powers down the USB peripheral.\nEnable the USB peripheral.\nGet whether the STALL condition is set for an endpoint.\nEnable or disable an endpoint.\nSet or clear the STALL condition for an endpoint.\nEndpoint’s type.\nSimulate a disconnect from the USB bus, causing the host …\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nConstructs a new EndpointAddress with the given index and …\nGets the index part of the endpoint address.\nGet the endpoint address\nPolling interval, in milliseconds.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nReturns true if the direction is IN, otherwise false.\nReturns true if the direction is OUT, otherwise false.\nMaximum packet size for the control pipe\nMax packet size, in bytes.\nWait for a bus-related event.\nRead a single packet of data from the endpoint, and return …\nReject a control request.\nInitiate a remote wakeup of the host by the device.\nRead a single setup packet from the endpoint.\nStart operation of the USB device.\nWait for the endpoint to be enabled.\nWrite a single packet of data to the endpoint.")