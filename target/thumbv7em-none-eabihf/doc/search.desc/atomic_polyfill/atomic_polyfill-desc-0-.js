searchState.loadedDescShard("atomic_polyfill", 0, "⚠️ THIS CRATE IS DEPRECATED ⚠️\nHas the effects of both <code>Acquire</code> and <code>Release</code> together: For …\nWhen coupled with a load, if the loaded value was written …\nA boolean type which can be safely shared between threads.\nAn integer type which can be safely shared between threads.\nAn integer type which can be safely shared between threads.\nAn integer type which can be safely shared between threads.\nAn integer type which can be safely shared between threads.\nA raw pointer type which can be safely shared between …\nAn integer type which can be safely shared between threads.\nAn integer type which can be safely shared between threads.\nAn integer type which can be safely shared between threads.\nAn integer type which can be safely shared between threads.\nAtomic memory orderings\nNo ordering constraints, only atomic operations.\nWhen coupled with a store, all previous operations become …\nLike <code>Acquire</code>/<code>Release</code>/<code>AcqRel</code> (for load, store, and …\nReturns a mutable pointer to the underlying <code>bool</code>.\nReturns a mutable pointer to the underlying pointer.\nReturns a mutable pointer to the underlying integer.\nReturns a mutable pointer to the underlying integer.\nReturns a mutable pointer to the underlying integer.\nReturns a mutable pointer to the underlying integer.\nReturns a mutable pointer to the underlying integer.\nReturns a mutable pointer to the underlying integer.\nReturns a mutable pointer to the underlying integer.\nReturns a mutable pointer to the underlying integer.\nStores a value into the <code>bool</code> if the current value is the …\nStores a value into the pointer if the current value is …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the <code>bool</code> if the current value is the …\nStores a value into the pointer if the current value is …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the <code>bool</code> if the current value is the …\nStores a value into the pointer if the current value is …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nStores a value into the atomic integer if the current …\nA compiler memory fence.\nCreates an <code>AtomicBool</code> initialized to <code>false</code>.\nCreates a null <code>AtomicPtr&lt;T&gt;</code>.\nAn atomic fence.\nAdds to the current value, returning the previous value.\nAdds to the current value, returning the previous value.\nAdds to the current value, returning the previous value.\nAdds to the current value, returning the previous value.\nAdds to the current value, returning the previous value.\nAdds to the current value, returning the previous value.\nAdds to the current value, returning the previous value.\nAdds to the current value, returning the previous value.\nLogical “and” with a boolean value.\nPerforms a bitwise “and” operation on the address of …\nBitwise “and” with the current value.\nBitwise “and” with the current value.\nBitwise “and” with the current value.\nBitwise “and” with the current value.\nBitwise “and” with the current value.\nBitwise “and” with the current value.\nBitwise “and” with the current value.\nBitwise “and” with the current value.\nOffsets the pointer’s address by adding <code>val</code> <em>bytes</em>, …\nOffsets the pointer’s address by subtracting <code>val</code> <em>bytes</em>, …\nMaximum with the current value.\nMaximum with the current value.\nMaximum with the current value.\nMaximum with the current value.\nMaximum with the current value.\nMaximum with the current value.\nMaximum with the current value.\nMaximum with the current value.\nMinimum with the current value.\nMinimum with the current value.\nMinimum with the current value.\nMinimum with the current value.\nMinimum with the current value.\nMinimum with the current value.\nMinimum with the current value.\nMinimum with the current value.\nLogical “nand” with a boolean value.\nBitwise “nand” with the current value.\nBitwise “nand” with the current value.\nBitwise “nand” with the current value.\nBitwise “nand” with the current value.\nBitwise “nand” with the current value.\nBitwise “nand” with the current value.\nBitwise “nand” with the current value.\nBitwise “nand” with the current value.\nLogical “not” with a boolean value.\nLogical “or” with a boolean value.\nPerforms a bitwise “or” operation on the address of …\nBitwise “or” with the current value.\nBitwise “or” with the current value.\nBitwise “or” with the current value.\nBitwise “or” with the current value.\nBitwise “or” with the current value.\nBitwise “or” with the current value.\nBitwise “or” with the current value.\nBitwise “or” with the current value.\nOffsets the pointer’s address by adding <code>val</code> (in units of …\nOffsets the pointer’s address by subtracting <code>val</code> (in …\nSubtracts from the current value, returning the previous …\nSubtracts from the current value, returning the previous …\nSubtracts from the current value, returning the previous …\nSubtracts from the current value, returning the previous …\nSubtracts from the current value, returning the previous …\nSubtracts from the current value, returning the previous …\nSubtracts from the current value, returning the previous …\nSubtracts from the current value, returning the previous …\nFetches the value, and applies a function to it that …\nFetches the value, and applies a function to it that …\nFetches the value, and applies a function to it that …\nFetches the value, and applies a function to it that …\nFetches the value, and applies a function to it that …\nFetches the value, and applies a function to it that …\nFetches the value, and applies a function to it that …\nFetches the value, and applies a function to it that …\nFetches the value, and applies a function to it that …\nFetches the value, and applies a function to it that …\nLogical “xor” with a boolean value.\nPerforms a bitwise “xor” operation on the address of …\nBitwise “xor” with the current value.\nBitwise “xor” with the current value.\nBitwise “xor” with the current value.\nBitwise “xor” with the current value.\nBitwise “xor” with the current value.\nBitwise “xor” with the current value.\nBitwise “xor” with the current value.\nBitwise “xor” with the current value.\nReturns the argument unchanged.\nReturns the argument unchanged.\nConverts a <code>bool</code> into an <code>AtomicBool</code>.\nReturns the argument unchanged.\nConverts a <code>*mut T</code> into an <code>AtomicPtr&lt;T&gt;</code>.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nConverts an <code>i8</code> into an <code>AtomicI8</code>.\nConverts an <code>u8</code> into an <code>AtomicU8</code>.\nReturns the argument unchanged.\nConverts an <code>i16</code> into an <code>AtomicI16</code>.\nReturns the argument unchanged.\nConverts an <code>u16</code> into an <code>AtomicU16</code>.\nReturns the argument unchanged.\nConverts an <code>i32</code> into an <code>AtomicI32</code>.\nReturns the argument unchanged.\nConverts an <code>u32</code> into an <code>AtomicU32</code>.\nReturns the argument unchanged.\nConverts an <code>isize</code> into an <code>AtomicIsize</code>.\nReturns the argument unchanged.\nConverts an <code>usize</code> into an <code>AtomicUsize</code>.\nReturns the argument unchanged.\nGet atomic access to a <code>&amp;mut bool</code>.\nGet atomic access to a pointer.\nGet atomic access to a <code>&amp;mut i8</code>.\nGet atomic access to a <code>&amp;mut u8</code>.\nGet atomic access to a <code>&amp;mut i16</code>.\nGet atomic access to a <code>&amp;mut u16</code>.\nGet atomic access to a <code>&amp;mut i32</code>.\nGet atomic access to a <code>&amp;mut u32</code>.\nGet atomic access to a <code>&amp;mut isize</code>.\nGet atomic access to a <code>&amp;mut usize</code>.\nGet atomic access to a <code>&amp;mut [bool]</code> slice.\nGet atomic access to a slice of pointers.\nGet atomic access to a <code>&amp;mut [i8]</code> slice.\nGet atomic access to a <code>&amp;mut [u8]</code> slice.\nGet atomic access to a <code>&amp;mut [i16]</code> slice.\nGet atomic access to a <code>&amp;mut [u16]</code> slice.\nGet atomic access to a <code>&amp;mut [i32]</code> slice.\nGet atomic access to a <code>&amp;mut [u32]</code> slice.\nGet atomic access to a <code>&amp;mut [isize]</code> slice.\nGet atomic access to a <code>&amp;mut [usize]</code> slice.\nCreates a new <code>AtomicBool</code> from a pointer.\nCreates a new <code>AtomicPtr</code> from a pointer.\nCreates a new reference to an atomic integer from a …\nCreates a new reference to an atomic integer from a …\nCreates a new reference to an atomic integer from a …\nCreates a new reference to an atomic integer from a …\nCreates a new reference to an atomic integer from a …\nCreates a new reference to an atomic integer from a …\nCreates a new reference to an atomic integer from a …\nCreates a new reference to an atomic integer from a …\nReturns a mutable reference to the underlying <code>bool</code>.\nReturns a mutable reference to the underlying pointer.\nReturns a mutable reference to the underlying integer.\nReturns a mutable reference to the underlying integer.\nReturns a mutable reference to the underlying integer.\nReturns a mutable reference to the underlying integer.\nReturns a mutable reference to the underlying integer.\nReturns a mutable reference to the underlying integer.\nReturns a mutable reference to the underlying integer.\nReturns a mutable reference to the underlying integer.\nGet non-atomic access to a <code>&amp;mut [AtomicBool]</code> slice.\nGet non-atomic access to a <code>&amp;mut [AtomicPtr]</code> slice.\nGet non-atomic access to a <code>&amp;mut [AtomicI8]</code> slice\nGet non-atomic access to a <code>&amp;mut [AtomicU8]</code> slice\nGet non-atomic access to a <code>&amp;mut [AtomicI16]</code> slice\nGet non-atomic access to a <code>&amp;mut [AtomicU16]</code> slice\nGet non-atomic access to a <code>&amp;mut [AtomicI32]</code> slice\nGet non-atomic access to a <code>&amp;mut [AtomicU32]</code> slice\nGet non-atomic access to a <code>&amp;mut [AtomicIsize]</code> slice\nGet non-atomic access to a <code>&amp;mut [AtomicUsize]</code> slice\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nConsumes the atomic and returns the contained value.\nConsumes the atomic and returns the contained value.\nConsumes the atomic and returns the contained value.\nConsumes the atomic and returns the contained value.\nConsumes the atomic and returns the contained value.\nConsumes the atomic and returns the contained value.\nConsumes the atomic and returns the contained value.\nConsumes the atomic and returns the contained value.\nConsumes the atomic and returns the contained value.\nConsumes the atomic and returns the contained value.\nLoads a value from the bool.\nLoads a value from the pointer.\nLoads a value from the atomic integer.\nLoads a value from the atomic integer.\nLoads a value from the atomic integer.\nLoads a value from the atomic integer.\nLoads a value from the atomic integer.\nLoads a value from the atomic integer.\nLoads a value from the atomic integer.\nLoads a value from the atomic integer.\nCreates a new <code>AtomicBool</code>.\nCreates a new <code>AtomicPtr</code>.\nCreates a new atomic integer.\nCreates a new atomic integer.\nCreates a new atomic integer.\nCreates a new atomic integer.\nCreates a new atomic integer.\nCreates a new atomic integer.\nCreates a new atomic integer.\nCreates a new atomic integer.\nStores a value into the bool.\nStores a value into the pointer.\nStores a value into the atomic integer.\nStores a value into the atomic integer.\nStores a value into the atomic integer.\nStores a value into the atomic integer.\nStores a value into the atomic integer.\nStores a value into the atomic integer.\nStores a value into the atomic integer.\nStores a value into the atomic integer.\nStores a value into the bool, returning the previous value.\nStores a value into the pointer, returning the previous …\nStores a value into the atomic integer, returning the …\nStores a value into the atomic integer, returning the …\nStores a value into the atomic integer, returning the …\nStores a value into the atomic integer, returning the …\nStores a value into the atomic integer, returning the …\nStores a value into the atomic integer, returning the …\nStores a value into the atomic integer, returning the …\nStores a value into the atomic integer, returning the …")