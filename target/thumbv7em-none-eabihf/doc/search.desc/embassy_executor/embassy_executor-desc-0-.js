searchState.loadedDescShard("embassy_executor", 0, "embassy-executor\nToo many instances of this task are already running.\nThread mode executor, using WFE/SEV.\nInterrupt mode executor.\nHandle to spawn tasks into an executor from any thread.\nError returned when spawning a task.\nToken to spawn a newly-created task in an executor.\nHandle to spawn tasks into an executor.\nGet a Spawner for the current executor.\nGet a Spawner for the current executor.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCreates a new <code>executor</code> instance and declares an …\nConvert this Spawner to a SendSpawner. This allows you to …\nSpawn a task into an executor, panicking on failure.\nSpawn a task into an executor, panicking on failure.\nCreate a new Executor.\nCreate a new, not started <code>InterruptExecutor</code>.\nReturn a SpawnToken that represents a failed spawn.\nExecutor interrupt callback.\nRaw executor.\nRun the executor.\nSpawn a task into an executor.\nSpawn a task into an executor.\nGet a SendSpawner for this executor\nStart the executor.\nDeclares an async task that can be run by <code>embassy-executor</code>…\nAn uninitialized <code>TaskStorage</code>.\nRaw executor.\nRaw storage that can hold up to N tasks of the same type.\nThis is essentially a <code>&amp;&#39;static TaskStorage&lt;F&gt;</code> where the …\nRaw storage in which a task can be spawned.\nTry to claim a <code>TaskStorage</code>.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nReturns the argument unchanged.\nInitialize the <code>TaskStorage</code> to run the given future.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCalls <code>U::from(self)</code>.\nCreate a new TaskStorage, in not-spawned state.\nCreate a new TaskPool, with all tasks in non-spawned state.\nCreate a new executor.\nPoll all queued tasks in this executor.\nTry to spawn the task.\nTry to spawn a task in the pool.\nGet a spawner that spawns tasks in this executor.\nGet a task pointer from a waker.\nWake a task by <code>TaskRef</code>.\nWake a task by <code>TaskRef</code> without calling pend.")