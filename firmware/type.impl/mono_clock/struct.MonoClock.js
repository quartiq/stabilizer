(function() {
    var type_impls = Object.fromEntries([["stabilizer",[["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-Clock-for-MonoClock%3CT,+HZ%3E\" class=\"impl\"><a href=\"#impl-Clock-for-MonoClock%3CT,+HZ%3E\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;T, const HZ: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.84.1/core/primitive.u32.html\">u32</a>&gt; <a class=\"trait\" href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/clock/trait.Clock.html\" title=\"trait embedded_time::clock::Clock\">Clock</a> for MonoClock&lt;T, HZ&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.84.1/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> + <a class=\"trait\" href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/time_int/trait.TimeInt.html\" title=\"trait embedded_time::time_int::TimeInt\">TimeInt</a> + <a class=\"trait\" href=\"https://doc.rust-lang.org/1.84.1/core/hash/trait.Hash.html\" title=\"trait core::hash::Hash\">Hash</a>,</div></h3></section></summary><div class=\"impl-items\"><details class=\"toggle\" open><summary><section id=\"associatedconstant.SCALING_FACTOR\" class=\"associatedconstant trait-impl\"><a href=\"#associatedconstant.SCALING_FACTOR\" class=\"anchor\">§</a><h4 class=\"code-header\">const <a href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/clock/trait.Clock.html#associatedconstant.SCALING_FACTOR\" class=\"constant\">SCALING_FACTOR</a>: <a class=\"struct\" href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/fraction/struct.Fraction.html\" title=\"struct embedded_time::fraction::Fraction\">Fraction</a> = _</h4></section></summary><div class='docblock'>The duration of one clock tick in seconds, AKA the clock precision.</div></details><details class=\"toggle\" open><summary><section id=\"associatedtype.T\" class=\"associatedtype trait-impl\"><a href=\"#associatedtype.T\" class=\"anchor\">§</a><h4 class=\"code-header\">type <a href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/clock/trait.Clock.html#associatedtype.T\" class=\"associatedtype\">T</a> = T</h4></section></summary><div class='docblock'>The type to hold the tick count</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.try_now\" class=\"method trait-impl\"><a href=\"#method.try_now\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/clock/trait.Clock.html#tymethod.try_now\" class=\"fn\">try_now</a>(&amp;self) -&gt; <a class=\"enum\" href=\"https://doc.rust-lang.org/1.84.1/core/result/enum.Result.html\" title=\"enum core::result::Result\">Result</a>&lt;<a class=\"struct\" href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/instant/struct.Instant.html\" title=\"struct embedded_time::instant::Instant\">Instant</a>&lt;MonoClock&lt;T, HZ&gt;&gt;, <a class=\"enum\" href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/clock/enum.Error.html\" title=\"enum embedded_time::clock::Error\">Error</a>&gt;</h4></section></summary><div class='docblock'>Get the current Instant <a href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/clock/trait.Clock.html#tymethod.try_now\">Read more</a></div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.new_timer\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"https://docs.rs/embedded-time/0.12.1/src/embedded_time/clock.rs.html#51-56\">Source</a><a href=\"#method.new_timer\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/clock/trait.Clock.html#method.new_timer\" class=\"fn\">new_timer</a>&lt;Dur&gt;(&amp;self, duration: Dur) -&gt; <a class=\"struct\" href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/timer/struct.Timer.html\" title=\"struct embedded_time::timer::Timer\">Timer</a>&lt;'_, <a class=\"struct\" href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/timer/param/struct.OneShot.html\" title=\"struct embedded_time::timer::param::OneShot\">OneShot</a>, <a class=\"struct\" href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/timer/param/struct.Armed.html\" title=\"struct embedded_time::timer::param::Armed\">Armed</a>, Self, Dur&gt;<div class=\"where\">where\n    Dur: <a class=\"trait\" href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/duration/trait.Duration.html\" title=\"trait embedded_time::duration::Duration\">Duration</a> + <a class=\"trait\" href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/fixed_point/trait.FixedPoint.html\" title=\"trait embedded_time::fixed_point::FixedPoint\">FixedPoint</a>,</div></h4></section></summary><div class='docblock'>Spawn a new, <code>OneShot</code> <a href=\"https://docs.rs/embedded-time/0.12.1/embedded_time/timer/struct.Timer.html\" title=\"struct embedded_time::timer::Timer\"><code>Timer</code></a> from this clock</div></details></div></details>","Clock","stabilizer::hardware::SystemTimer"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-Clone-for-MonoClock%3CT,+HZ%3E\" class=\"impl\"><a href=\"#impl-Clone-for-MonoClock%3CT,+HZ%3E\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;T, const HZ: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.84.1/core/primitive.u32.html\">u32</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.84.1/core/clone/trait.Clone.html\" title=\"trait core::clone::Clone\">Clone</a> for MonoClock&lt;T, HZ&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.84.1/core/clone/trait.Clone.html\" title=\"trait core::clone::Clone\">Clone</a>,</div></h3></section></summary><div class=\"impl-items\"><details class=\"toggle method-toggle\" open><summary><section id=\"method.clone\" class=\"method trait-impl\"><a href=\"#method.clone\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.84.1/core/clone/trait.Clone.html#tymethod.clone\" class=\"fn\">clone</a>(&amp;self) -&gt; MonoClock&lt;T, HZ&gt;</h4></section></summary><div class='docblock'>Returns a copy of the value. <a href=\"https://doc.rust-lang.org/1.84.1/core/clone/trait.Clone.html#tymethod.clone\">Read more</a></div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.clone_from\" class=\"method trait-impl\"><span class=\"rightside\"><span class=\"since\" title=\"Stable since Rust version 1.0.0\">1.0.0</span> · <a class=\"src\" href=\"https://doc.rust-lang.org/1.84.1/src/core/clone.rs.html#174\">Source</a></span><a href=\"#method.clone_from\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.84.1/core/clone/trait.Clone.html#method.clone_from\" class=\"fn\">clone_from</a>(&amp;mut self, source: &amp;Self)</h4></section></summary><div class='docblock'>Performs copy-assignment from <code>source</code>. <a href=\"https://doc.rust-lang.org/1.84.1/core/clone/trait.Clone.html#method.clone_from\">Read more</a></div></details></div></details>","Clone","stabilizer::hardware::SystemTimer"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-Debug-for-MonoClock%3CT,+HZ%3E\" class=\"impl\"><a href=\"#impl-Debug-for-MonoClock%3CT,+HZ%3E\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;T, const HZ: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.84.1/core/primitive.u32.html\">u32</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.84.1/core/fmt/trait.Debug.html\" title=\"trait core::fmt::Debug\">Debug</a> for MonoClock&lt;T, HZ&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.84.1/core/fmt/trait.Debug.html\" title=\"trait core::fmt::Debug\">Debug</a>,</div></h3></section></summary><div class=\"impl-items\"><details class=\"toggle method-toggle\" open><summary><section id=\"method.fmt\" class=\"method trait-impl\"><a href=\"#method.fmt\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.84.1/core/fmt/trait.Debug.html#tymethod.fmt\" class=\"fn\">fmt</a>(&amp;self, f: &amp;mut <a class=\"struct\" href=\"https://doc.rust-lang.org/1.84.1/core/fmt/struct.Formatter.html\" title=\"struct core::fmt::Formatter\">Formatter</a>&lt;'_&gt;) -&gt; <a class=\"enum\" href=\"https://doc.rust-lang.org/1.84.1/core/result/enum.Result.html\" title=\"enum core::result::Result\">Result</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.84.1/core/primitive.unit.html\">()</a>, <a class=\"struct\" href=\"https://doc.rust-lang.org/1.84.1/core/fmt/struct.Error.html\" title=\"struct core::fmt::Error\">Error</a>&gt;</h4></section></summary><div class='docblock'>Formats the value using the given formatter. <a href=\"https://doc.rust-lang.org/1.84.1/core/fmt/trait.Debug.html#tymethod.fmt\">Read more</a></div></details></div></details>","Debug","stabilizer::hardware::SystemTimer"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-MonoClock%3CT,+HZ%3E\" class=\"impl\"><a href=\"#impl-MonoClock%3CT,+HZ%3E\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;T, const HZ: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.84.1/core/primitive.u32.html\">u32</a>&gt; MonoClock&lt;T, HZ&gt;</h3></section></summary><div class=\"impl-items\"><details class=\"toggle method-toggle\" open><summary><section id=\"method.new\" class=\"method\"><h4 class=\"code-header\">pub fn <a class=\"fn\">new</a>(now: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.84.1/core/primitive.fn.html\">fn</a>() -&gt; T) -&gt; MonoClock&lt;T, HZ&gt;</h4></section></summary><div class=\"docblock\"><p>! Create a new <code>MonoClock</code> using e.g. RTIC’s <code>monotonics::now()</code>.\n!\n! Args:\n! * now: a closure that returns the current ticks\n!\n! <code>! // In your `app` `init()`, set up a `Monotonic` as usual, e.g.: ! use mono_clock::MonoClock, systick_monotonic::Systick; ! const HZ: u32 = 1_000; ! let sysclk = 400_000_000u32; ! let mono = Systick::&lt;HZ&gt;::new(c.core.SYST, sysclk); ! // Then build a `Clock` that is `Copy` and can be passed ! // around by value or reference: ! let clock = MonoClock::&lt;u32, HZ&gt;::new(|| monotonics::now()); !</code></p>\n</div></details></div></details>",0,"stabilizer::hardware::SystemTimer"],["<section id=\"impl-Copy-for-MonoClock%3CT,+HZ%3E\" class=\"impl\"><a href=\"#impl-Copy-for-MonoClock%3CT,+HZ%3E\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;T, const HZ: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.84.1/core/primitive.u32.html\">u32</a>&gt; <a class=\"trait\" href=\"https://doc.rust-lang.org/1.84.1/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> for MonoClock&lt;T, HZ&gt;<div class=\"where\">where\n    T: <a class=\"trait\" href=\"https://doc.rust-lang.org/1.84.1/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a>,</div></h3></section>","Copy","stabilizer::hardware::SystemTimer"]]]]);
    if (window.register_type_impls) {
        window.register_type_impls(type_impls);
    } else {
        window.pending_type_impls = type_impls;
    }
})()
//{"start":55,"fragment_lengths":[11016]}