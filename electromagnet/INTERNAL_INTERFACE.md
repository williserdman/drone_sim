# Electromagnet Internal Interface

`ScenarioPolicy.observe_clock(timestamp_ns)` is the pure same-process seam. It
returns at most one immutable inactive event, rejects invalid or regressing
simulation time, and cannot repair after failure.

`ScenarioController` owns publication, structured evidence, and quiescence.
`runtime_node` alone imports ROS 2 and translates the immutable event into the
public message.

Future scenario-policy and ROS 2 adapter submodules may communicate through documented language-native seams. Scenario policy must be a deterministic transformation of run-scoped, simulation-timestamped input state.
