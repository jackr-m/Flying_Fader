searchState.loadedDescShard("embassy_hal_internal", 0, "embassy-hal-internal\nPeripheral singleton type\nTrait for any type that can be used as a peripheral of …\nAn exclusive reference to a peripheral.\nAtomic reusable ringbuffer.\nUnsafely clone (duplicate) a peripheral singleton.\nUnsafely clone (duplicate) a peripheral singleton.\nTypes for controlling when drop is invoked.\nReturns the argument unchanged.\nImplement the peripheral trait.\nInterrupt handling for cortex-m devices.\nGenerate a standard <code>mod interrupt</code> for a HAL.\nCalls <code>U::from(self)</code>.\nConvert a value into a <code>PeripheralRef</code>.\nConvert a value into a <code>PeripheralRef</code>.\nConvenience converting into reference.\nMap the inner peripheral using <code>Into</code>.\nCreate a new reference to a peripheral.\nDefining peripheral type.\nTypes for the peripheral singletons.\nDefine the peripherals struct.\nTypes for dealing with rational numbers.\nReborrow into a “child” PeripheralRef.\nA type which can only read from a ring buffer.\nAtomic reusable ringbuffer\nA type which can only write to a ring buffer.\nDeinitialize the ringbuffer.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nInitialize the ring buffer with a buffer.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCheck if buffer is empty.\nCheck if buffer is full.\nReturn length of buffer.\nCreate a new empty ringbuffer.\nPop data from the buffer in-place.\nGet a buffer where data can be popped from.\nMark n bytes as read and allow advance the read index.\nPop one data byte.\nGet a buffer where data can be popped from.\nPush data into the buffer in-place.\nGet a buffer where data can be pushed to.\nGet up to two buffers where data can be pushed to.\nMark n bytes as written and advance the write index.\nPush one data byte.\nGet a buffer where data can be pushed to.\nGet up to two buffers where data can be pushed to.\nCreate a reader.\nCreate a writer.\nAn explosive ordinance that panics if it is improperly …\nA type to delay the drop handler invocation.\nPrevent drop handler from running.\nDefuses the bomb, rendering it safe to drop.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCreate a new instance.\nCreate a new instance.\nRepresents an interrupt type that can be configured by …\nThe interrupt priority level.\nDisable the interrupt.\nEnable the interrupt.\nReturns the argument unchanged.\nGet the priority of the interrupt.\nCalls <code>U::from(self)</code>.\nCheck if interrupt is being handled.\nCheck if interrupt is enabled.\nCheck if interrupt is pending.\nSet interrupt pending.\nSet the interrupt priority.\nSet the interrupt priority with an already-acquired …\nUnset interrupt pending.\nRepresents the ratio between two numbers.\nGets an immutable reference to the denominator.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCreates a new <code>Ratio</code>.\nGets an immutable reference to the numerator.\nConverts to an integer, rounding towards zero.")