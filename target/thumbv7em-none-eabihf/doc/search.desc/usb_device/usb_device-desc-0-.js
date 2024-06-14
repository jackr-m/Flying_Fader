searchState.loadedDescShard("usb_device", 0, "Experimental device-side USB stack for embedded devices.\nA buffer too short for the data to read was passed, or …\nClasses attempted to allocate more packet buffer memory …\nClasses attempted to allocate more endpoints than the …\nContains the error value\nDevice to host (IN)\nThe endpoint address is invalid or already used.\nOperation is not valid in the current state of the object.\nContains the success value\nHost to device (OUT)\nParsing failed due to invalid input.\nResult for USB operations.\nOperation is not supported by device or configuration.\nDirection of USB traffic. Note that in the USB standard …\nA USB stack error.\nAn operation would block because the device is currently …\nFor implementing peripheral drivers.\nFor implementing standard as well as vendor-specific USB …\nPrelude for class implementors.\nUSB control transfers and the SETUP packet.\nCreating USB descriptors\nUSB composite device.\nUSB endpoints.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nPrelude for device implementors.\nTest USB class for testing USB driver implementations. …\nUSB packets have been received or sent. Each data field is …\nA handle for a USB interface that contains its number.\nNo events or packets to report.\nEvent and incoming packet information returned by …\nIndicates that <code>set_device_address</code> must be called before …\nThe USB reset condition has been detected.\nA USB resume request has been detected after being …\nA handle for a USB string descriptor that contains its …\nA USB suspend request has been detected or, in the case of …\nA trait for device-specific USB peripherals. Implement …\nHelper type used for UsbBus resource allocation and …\nAllocates an endpoint with the specified direction and …\nAllocates an endpoint and specified endpoint parameters. …\nAllocates a bulk endpoint.\nAllocates a control endpoint.\nEnables and initializes the USB peripheral. Soon after …\nSimulates a disconnect from the USB bus, causing the host …\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nAllocates a new interface number.\nAllocates an interrupt endpoint.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nGets whether the STALL condition is set for an endpoint.\nAllocates an isochronous endpoint.\nCreates a new <code>UsbBusAllocator</code> that wraps the provided …\nGets information about events and incoming data. Usually …\nReads a single packet of data from the specified endpoint …\nCalled when the host resets the device. This will be soon …\nResumes from suspend mode. This may only be called after …\nSets the device USB address to <code>addr</code>.\nSets or clears the STALL condition for an endpoint. If the …\nAllocates a new string index.\nCauses the USB peripheral to enter USB suspend mode, …\nWrites a single packet of data to the specified endpoint …\nAn IN packet has finished transmitting. This event should …\nAn OUT packet has been received. This event should …\nA SETUP packet has been received. This event should …\nHandle for a control IN transfer. When implementing a …\nHandle for a control OUT transfer. When implementing a …\nA trait for implementing USB classes.\nAccepts the transfer with a callback that can write to the …\nAccepts the transfer by succesfully responding to the …\nAccepts the transfer with the supplied buffer.\nAccepts the transfer with the supplied static buffer. This …\nCalled when a control request is received with direction …\nCalled when a control request is received with direction …\nGets the data from the data stage of the request. May be …\nCalled when endpoint with address <code>addr</code> has completed …\nCalled when endpoint with address <code>addr</code> has received data …\nCalled when endpoint with address <code>addr</code> has received a …\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalled when the interfaces alternate setting state is …\nCalled when a GET_DESCRIPTOR request is received for a BOS …\nCalled when a GET_DESCRIPTOR request is received for a …\nGets a class-specific string descriptor.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalled whenever the <code>UsbDevice</code> is polled.\nRejects the transfer by stalling the pipe.\nRejects the transfer by stalling the pipe.\nGets the request from the SETUP packet.\nGets the request from the SETUP packet.\nCalled after a USB reset after the bus reset sequence is …\nCalled when the interfaces alternate setting state is …\nStandard USB control request Clear Feature\nRequest is intended for a USB class.\nRequest is intended for the entire device.\nRequest is intended for an endpoint. Generally, the <code>index</code> …\nStandard USB feature Device Remote Wakeup for Set/Clear …\nStandard USB feature Endpoint Halt for Set/Clear Feature\nStandard USB control request Get Configuration\nStandard USB control request Get Descriptor\nStandard USB control request Get Interface\nStandard USB control request Get Status\nRequest is intended for an interface. Generally, the <code>index</code> …\nNone of the above.\nControl request recipient.\nA control request read from a SETUP packet.\nControl request type.\nReserved.\nReserved.\nStandard USB control request Set Address\nStandard USB control request Set Configuration\nStandard USB control request Set Descriptor\nStandard USB control request Set Feature\nStandard USB control request Set Interface\nStandard USB control request Synch Frame\nRequest is a USB standard request. Usually handled by …\nRequest is vendor-specific.\nGets the descriptor type and index from the value field of …\nDirection of the request.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nRequest index. The meaning of the value depends on the …\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nLength of the DATA stage. For control OUT transfers this …\nRecipient of the request.\nRequest code. The meaning of the value depends on the …\nType of the request.\nRequest value. The meaning of the value depends on the …\nA writer for Binary Object Store descriptor.\nA writer for USB descriptors.\nWrites capability descriptor to a BOS\nStandard capability descriptor types\nStandard descriptor types\nWrites an endpoint descriptor.\nWrites an endpoint descriptor with extra trailing data.\nReturns the argument unchanged.\nReturns the argument unchanged.\nWrites a interface association descriptor. Call from …\nWrites a interface descriptor.\nWrites a interface descriptor with a specific alternate …\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nString descriptor language IDs.\nGets the current position in the buffer, i.e. the number …\nWrites an arbitrary (usually class-specific) descriptor.\nWrites an arbitrary (usually class-specific) descriptor by …\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nThe USB device has received an address from the host.\nThe bConfiguration value for the not configured state.\nThe bConfiguration value for the single configuration …\nThe USB device has been configured and is fully functional.\nThe default value for bAlternateSetting for all interfaces.\nThe USB device has just been created or reset.\nProvides basic string descriptors about the device, …\nThe USB device has been suspended by the host or it has …\nUSB 2.0 compliance\nUSB 2.1 compliance.\nA USB device consisting of one or more device classes.\nUsed to build new <code>UsbDevice</code>s.\nThe global state of the USB device.\nUsb spec revision.\nA USB vendor ID and product ID pair.\nCreates the <code>UsbDevice</code> instance with the configuration in …\nGets a reference to the <code>UsbBus</code> implementation used by this …\nConfigures the device as a composite device with interface …\nSets the device class code assigned by USB.org. Set to <code>0xff</code>…\nSets the device protocol code. Depends on class and …\nSets the device release version in BCD.\nSets the device sub-class code. Depends on class.\nSimulates a disconnect from the USB bus, causing the host …\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nSpecify the manufacturer name for this language.\nSets the maximum packet size in bytes for the control …\nSets the maximum current drawn from the USB bus by the …\nCreates a builder for constructing a new <code>UsbDevice</code>.\nCreate a new descriptor list with the provided language.\nPolls the <code>UsbBus</code> for new events and dispatches them to the …\nSpecify the product name for this language.\nGets whether host remote wakeup has been enabled by the …\nGets whether the device is currently self powered.\nSets whether the device may have an external power source.\nSpecify the serial number for this language.\nSets whether the device is currently self powered.\nGets the current state of the device.\nSpecify the strings for the device.\nSets whether the device supports remotely waking up the …\nSets which Usb 2 revision to comply to.\nSource sample clock is locked to Sink, Sink sample clock …\nSource and Sink sample clocks are free running.\nBulk endpoint. Used for large amounts of best-effort …\nControl endpoint. Used for device management. Only the …\nDirection value of the marker type.\nEndpoint is used for isochronous data.\nHandle for a USB endpoint. The endpoint direction is …\nType-safe endpoint address.\nTrait for endpoint type markers.\nA device-to-host (IN) endpoint.\nA host-to-device (OUT) endpoint.\nUSB endpoint transfer type.\nFeedback for synchronization.\nEndpoint is data and provides implicit feedback for …\nMarker type for IN endpoints.\nInterrupt endpoint. Used for small amounts of …\nIsochronous endpoint. Used for time-critical unreliable …\nIsochronous transfers employ one of three synchronization …\nIntended use of an isochronous endpoint, see USB 2.0 spec …\nSynchronization is not implemented for this endpoint.\nMarker type for OUT endpoints.\nSource and Sink sample clocks are locked to USB SOF.\nGets the endpoint address including direction bit.\nGets the direction part of the address.\nGets the endpoint transfer type.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nConstructs a new EndpointAddress with the given index and …\nGets the index part of the endpoint address.\nGets the poll interval for interrupt endpoints.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nReturns true if the direction is IN, otherwise false.\nReturns true if the direction is OUT, otherwise false.\nGets the maximum packet size for the endpoint.\nReads a single packet of data from the specified endpoint …\nSets the STALL condition for the endpoint.\nFormat EndpointType for use in bmAttributes transfer type …\nClears the STALL condition of the endpoint.\nWrites a single packet of data to the specified endpoint …\nSynchronization model used for the data stream that this …\nEndpoint’s role in the synchronization model selected by …\nError type for the USB device builder\nControl endpoint can only be 8, 16, 32, or 64 byte max …\nConfiguration specifies higher USB power draw than allowed\nString descriptors were provided in more languages than …\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nTest USB class for testing USB driver implementations. …\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nConvenience method to create a UsbDevice that is …\nConvenience method to create a UsbDeviceBuilder that is …\nCreates a new TestClass.\nMust be called after polling the UsbDevice.")