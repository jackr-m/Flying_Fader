(function() {var implementors = {
"cortex_m":[],
"embassy_embedded_hal":[["impl&lt;'d, M, BUS, CS, BusErr, CsErr&gt; <a class=\"trait\" href=\"embedded_hal/blocking/spi/trait.Write.html\" title=\"trait embedded_hal::blocking::spi::Write\">Write</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/nightly/core/primitive.u8.html\">u8</a>&gt; for <a class=\"struct\" href=\"embassy_embedded_hal/shared_bus/blocking/spi/struct.SpiDevice.html\" title=\"struct embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice\">SpiDevice</a>&lt;'_, M, BUS, CS&gt;<div class=\"where\">where\n    M: RawMutex,\n    BUS: <a class=\"trait\" href=\"embedded_hal/blocking/spi/trait.Write.html\" title=\"trait embedded_hal::blocking::spi::Write\">Write</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/nightly/core/primitive.u8.html\">u8</a>, Error = BusErr&gt;,\n    CS: <a class=\"trait\" href=\"embedded_hal/digital/trait.OutputPin.html\" title=\"trait embedded_hal::digital::OutputPin\">OutputPin</a>&lt;Error = CsErr&gt;,</div>"]],
"embassy_stm32":[["impl&lt;'d, T: <a class=\"trait\" href=\"embassy_stm32/spi/trait.Instance.html\" title=\"trait embassy_stm32::spi::Instance\">Instance</a>, Tx, Rx&gt; <a class=\"trait\" href=\"embedded_hal/blocking/spi/trait.Write.html\" title=\"trait embedded_hal::blocking::spi::Write\">Write</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/nightly/core/primitive.u16.html\">u16</a>&gt; for <a class=\"struct\" href=\"embassy_stm32/spi/struct.Spi.html\" title=\"struct embassy_stm32::spi::Spi\">Spi</a>&lt;'d, T, Tx, Rx&gt;"],["impl&lt;'d, T: <a class=\"trait\" href=\"embassy_stm32/spi/trait.Instance.html\" title=\"trait embassy_stm32::spi::Instance\">Instance</a>, Tx, Rx&gt; <a class=\"trait\" href=\"embedded_hal/blocking/spi/trait.Write.html\" title=\"trait embedded_hal::blocking::spi::Write\">Write</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/nightly/core/primitive.u8.html\">u8</a>&gt; for <a class=\"struct\" href=\"embassy_stm32/spi/struct.Spi.html\" title=\"struct embassy_stm32::spi::Spi\">Spi</a>&lt;'d, T, Tx, Rx&gt;"]]
};if (window.register_implementors) {window.register_implementors(implementors);} else {window.pending_implementors = implementors;}})()