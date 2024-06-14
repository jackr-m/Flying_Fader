searchState.loadedDescShard("embassy_sync", 0, "embassy-sync\nBlocking mutex.\nA queue for sending values between asynchronous tasks.\nAsync mutex.\nSynchronization primitive for initializing a value once, …\nAsync byte stream pipe.\nA queue for sending values between asynchronous tasks.\nImplementation of PubSubChannel, a queue where published …\nA synchronization primitive for controlling access to a …\nA synchronization primitive for passing the latest value …\nAsync low-level wait queues\nA zero-copy queue for sending values between asynchronous …\nA mutex that allows borrowing data across executors and …\nBlocking mutex (not async)\nA mutex that allows borrowing data in the context of a …\nA “mutex” that only allows borrowing from thread mode.\nBorrows the data for the duration of the critical section\nBorrows the data\nBorrows the data\nCreates a new mutex based on a pre-existing raw mutex.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns a mutable reference to the underlying data.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nConsumes this mutex, returning the underlying data.\nCreates a critical section and grants temporary access to …\nLock the <code>ThreadModeMutex</code>, granting access to the data.\nCreates a new mutex in an unlocked state ready for use.\nCreates a new mutex\nMutex primitives.\nA mutex that allows borrowing data across executors and …\nCreate a new <code>RawMutex</code> instance.\nA mutex that allows borrowing data in the context of a …\nRaw mutex trait.\nA “mutex” that only allows borrowing from thread mode.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nLock this <code>RawMutex</code>.\nCreate a new <code>CriticalSectionRawMutex</code>.\nCreate a new <code>NoopRawMutex</code>.\nCreate a new <code>ThreadModeRawMutex</code>.\nA bounded channel for communicating between asynchronous …\nFuture returned by <code>DynamicReceiver::receive</code>.\nReceive-only access to a <code>Channel</code> without knowing channel …\nFuture returned by <code>DynamicSender::send</code>.\nSend-only access to a <code>Channel</code> without knowing channel size.\nA message could not be received because the channel is …\nThe data could not be sent on the channel because the …\nFuture returned by <code>Channel::receive</code> and  <code>Receiver::receive</code>.\nFuture returned by <code>Channel::ready_to_receive</code> and  …\nReceive-only access to a <code>Channel</code>.\nFuture returned by <code>Channel::send</code> and  <code>Sender::send</code>.\nSend-only access to a <code>Channel</code>.\nError returned by <code>try_receive</code>.\nError returned by <code>try_send</code>.\nReturns the maximum number of elements the channel can …\nClears all elements in the channel.\nGet a receiver for this channel using dynamic dispatch.\nGet a sender for this channel using dynamic dispatch.\nReturns the free capacity of the channel.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nReturns whether the channel is empty.\nReturns whether the channel is full.\nReturns the number of elements currently in the channel.\nEstablish a new bounded channel. For example, to create …\nAllows a poll_fn to poll until the channel is ready to …\nAllows a poll_fn to poll until the channel is ready to …\nAllows a poll_fn to poll until the channel is ready to …\nAllows a poll_fn to poll until the channel is ready to send\nAllows a poll_fn to poll until the channel is ready to send\nAllows a poll_fn to poll until the channel is ready to send\nPoll the channel for the next item\nPoll the channel for the next item\nPoll the channel for the next message\nIs a value ready to be received in the channel\nIs a value ready to be received in the channel\nReceive the next value.\nReceive the next value.\nReceive the next value.\nGet a receiver for this channel.\nSends a value.\nSends a value.\nSend a value, waiting until there is capacity.\nGet a sender for this channel.\nAttempt to immediately receive the next value.\nAttempt to immediately receive the next value.\nAttempt to immediately receive a message.\nAttempt to immediately send a message.\nAttempt to immediately send a message.\nAttempt to immediately send a message.\nA handle to a held <code>Mutex</code> that has had a function applied …\nAsync mutex.\nAsync mutex guard.\nError returned by <code>Mutex::try_lock</code>\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns a mutable reference to the underlying data.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nConsumes this mutex, returning the underlying data.\nLock the mutex.\nReturns a locked view over a portion of the locked data.\nReturns a locked view over a portion of the locked data.\nCreate a new mutex with the given value.\nAttempt to immediately lock the mutex.\nThe <code>OnceLock</code> is a synchronization primitive that allows for\nReturns the argument unchanged.\nGet a reference to the underlying value, waiting for it to …\nGet a reference to the underlying value, initializing it …\nSet the underlying value. If the value is already set, …\nCalls <code>U::from(self)</code>.\nConsume the <code>OnceLock</code>, returning the underlying value if it …\nCheck if the value has been set.\nCreate a new uninitialized <code>OnceLock</code>.\nTake the underlying value if it was initialized, …\nTry to get a reference to the underlying value if it …\nNo data could be read from the pipe because it is currently\nFuture returned by [<code>Pipe::fill_buf</code>] and  <code>Reader::fill_buf</code>.\nNo data could be written to the pipe because it is …\nA bounded byte-oriented pipe for communicating between …\nFuture returned by <code>Pipe::read</code> and  <code>Reader::read</code>.\nRead-only access to a <code>Pipe</code>.\nError returned by <code>try_read</code>.\nError returned by <code>try_write</code>.\nFuture returned by <code>Pipe::write</code> and  <code>Writer::write</code>.\nWrite-only access to a <code>Pipe</code>.\nTotal byte capacity.\nClear the data in the pipe’s buffer.\nTell this buffer that <code>amt</code> bytes have been consumed from …\nReturn the contents of the internal buffer, filling it …\nFree byte capacity.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nReturn whether the pipe is empty (no data buffered)\nReturn whether the pipe is full (no free space in the …\nUsed byte capacity.\nEstablish a new bounded pipe. For example, to create one …\nRead some bytes from the pipe.\nRead some bytes from the pipe.\nSplit this pipe into a BufRead-capable reader and a writer.\nTry returning contents of the internal buffer.\nAttempt to immediately read some bytes from the pipe.\nAttempt to immediately read some bytes from the pipe.\nAttempt to immediately write some bytes to the pipe.\nAttempt to immediately write some bytes to the pipe.\nWrite some bytes to the pipe.\nWrite some bytes to the pipe.\nWrite all bytes to the pipe.\nThe binary heap kind: min-heap or max-heap\nMax-heap\nMin-heap\nA bounded channel for communicating between asynchronous …\nFuture returned by <code>PriorityChannel::receive</code> and  …\nReceive-only access to a <code>PriorityChannel</code>.\nFuture returned by <code>PriorityChannel::send</code> and  <code>Sender::send</code>.\nSend-only access to a <code>PriorityChannel</code>.\nReturns the maximum number of elements the channel can …\nClears all elements in the channel.\nReturns the free capacity of the channel.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nReturns whether the channel is empty.\nReturns whether the channel is full.\nReturns the number of elements currently in the channel.\nEstablish a new bounded channel. For example, to create …\nAllows a poll_fn to poll until the channel is ready to …\nAllows a poll_fn to poll until the channel is ready to …\nAllows a poll_fn to poll until the channel is ready to send\nAllows a poll_fn to poll until the channel is ready to send\nPoll the channel for the next item\nPoll the channel for the next message\nReceive the next value.\nReceive the next value.\nGet a receiver for this channel.\nSends a value.\nSend a value, waiting until there is capacity.\nGet a sender for this channel.\nAttempt to immediately receive the next value.\nAttempt to immediately receive a message.\nAttempt to immediately send a message.\nAttempt to immediately send a message.\nError type for the PubSubChannel\nThe subscriber did not receive all messages and lagged by …\nAll publisher slots are used. To add another publisher, …\nAll subscriber slots are used. To add another subscriber, …\nA message was received\n‘Middle level’ behaviour of the pubsub channel. This …\nA broadcast channel implementation where multiple …\nThe result of the subscriber wait procedure\nReturns the maximum number of elements the channel can …\nClears all elements in the channel.\nCreate a new publisher that can only send immediate …\nCreate a new publisher\nCreate a new subscriber. It will only receive messages …\nReturns the free capacity of the channel.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCreate a new publisher that can only send immediate …\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nReturns whether the channel is empty.\nReturns whether the channel is full.\nReturns the number of elements currently in the channel.\nCreate a new channel\nImplementation of anything directly publisher related\nCreate a new publisher\nImplementation of anything directly subscriber related\nCreate a new subscriber. It will only receive messages …\nAn immediate publisher that holds a dynamic reference to …\nA publisher that holds a dynamic reference to the channel\nA publisher that can only use the <code>publish_immediate</code> …\nAn immediate publisher that holds a generic reference to …\nA publisher to a channel\nA publisher that holds a generic reference to the channel\nFuture for the publisher wait action\nReturns the maximum number of elements the <em><strong>channel</strong></em> can …\nReturns the maximum number of elements the <em><strong>channel</strong></em> can …\nClears all elements in the <em><strong>channel</strong></em>.\nClears all elements in the <em><strong>channel</strong></em>.\nReturns the free capacity of the <em><strong>channel</strong></em>.\nReturns the free capacity of the <em><strong>channel</strong></em>.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nReturns whether the <em><strong>channel</strong></em> is empty.\nReturns whether the <em><strong>channel</strong></em> is empty.\nReturns whether the <em><strong>channel</strong></em> is full.\nReturns whether the <em><strong>channel</strong></em> is full.\nReturns the number of elements currently in the <em><strong>channel</strong></em>.\nReturns the number of elements currently in the <em><strong>channel</strong></em>.\nPublish a message. But if the message queue is full, wait …\nPublish a message right now even when the queue is full. …\nPublish the message right now even when the queue is full. …\nPublish a message if there is space in the message queue\nPublish a message if there is space in the message queue\nA subscriber that holds a dynamic reference to the channel\nA subscriber to a channel\nA subscriber that holds a generic reference to the channel\nFuture for the subscriber wait action\nThe amount of messages this subscriber hasn’t received …\nReturns the maximum number of elements the <em><strong>channel</strong></em> can …\nClears all elements in the <em><strong>channel</strong></em>.\nReturns the free capacity of the <em><strong>channel</strong></em>.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nReturns whether the <em><strong>channel</strong></em> is empty.\nReturns whether the <em><strong>channel</strong></em> is full.\nReturns the number of elements currently in the <em><strong>channel</strong></em>. …\nWait for a published message\nWait for a published message (ignoring lag results)\nTry to see if there’s a published message we haven’t …\nTry to see if there’s a published message we haven’t …\nThe error returned when the semaphore is unable to acquire …\nA fair <code>Semaphore</code> implementation.\nA greedy <code>Semaphore</code> implementation.\nAn asynchronous semaphore.\nA representation of a number of acquired permits.\nAn error indicating the <code>FairSemaphore</code>’s wait queue is …\nAsynchronously acquire one or more permits from the …\nAsynchronously acquire all permits controlled by the …\nPrevent the acquired permits from being released on drop.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCreate a new <code>Semaphore</code>.\nCreate a new <code>FairSemaphore</code>.\nThe number of acquired permits.\nRelease <code>permits</code> back to the semaphore, making them …\nReset the number of available permints in the semaphore to …\nTry to immediately acquire one or more permits from the …\nTry to immediately acquire all available permits from the …\nSingle-slot signaling primitive.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCreate a new <code>Signal</code>.\nRemove the queued value in this <code>Signal</code>, if any.\nMark this Signal as signaled.\nnon-blocking method to check whether this signal has been …\nnon-blocking method to try and take the signal value.\nFuture that completes when this Signal has been signaled.\nUtility struct to register and wake a waker.\nUtility struct to register and wake multiple wakers.\nUtility struct to register and wake a waker.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCreate a new <code>AtomicWaker</code>.\nCreate a new empty instance\nCreate a new <code>WakerRegistration</code>.\nReturns true if a waker is currently registered\nRegister a waker. Overwrites the previous waker, if any.\nRegister a waker. If the buffer is full the function …\nRegister a waker. Overwrites the previous waker, if any.\nWake the registered waker, if any.\nWake all registered wakers. This clears the buffer\nWake the registered waker, if any.\nA bounded zero-copy channel for communicating between …\nReceive-only access to a <code>Channel</code>.\nSend-only access to a <code>Channel</code>.\nCreates one further <code>Sender</code> over the same channel.\nCreates one further <code>Sender</code> over the same channel.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nInitialize a new <code>Channel</code>.\nAttempts to asynchronously receive a value over the …\nAttempts to send a value over the channel.\nAsynchronously receive a value over the channel.\nNotify the channel that the receiving of the value has …\nAsynchronously send a value over the channel.\nNotify the channel that the sending of the value has been …\nCreates a <code>Sender</code> and <code>Receiver</code> from an existing channel.\nAttempts to receive a value over the channel.\nAttempts to send a value over the channel.")