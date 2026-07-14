# Stage4-B1 microphysics interface design

## Purpose

Freeze the current Stage4-B1 bearing-force implementation, then introduce one
transparent microphysics interface.  Future thermal, roughness, and impurity
models may operate only on a local contact-state contract.  With all switches
disabled, the 64-step transverse Stage4-B1 response must match the frozen
implementation exactly.

## Scope and invariants

- Preserve the existing 5-DOF bearing assembly and local force law.
- Do not modify Newmark integration variables, KKT constraints, global 192-DOF
  vectors, or global M/C/K matrices.
- Do not enable a thermal, roughness, or impurity constitutive model, and do
  not run a batch study.
- Retain a frozen copy of the pre-interface `nonlinear_bearing_force.m` for
  regression comparison.  It is callable only through a validation-specific
  function-resolution path, so Newmark separately exercises frozen and new
  implementations; it is never on the ordinary production search path.

## Architecture

`nonlinear_bearing_force.m` becomes an orchestration boundary.  Per bearing it
creates a local `contact_base`, calls the sole microphysics entry, then solves
the force from the returned local state:

```matlab
contact_base = build_base_contact_state(...);
[contact_mod, micro_state] = apply_microphysics( ...
    contact_base, operating_state, micro_state, cfg);
[f5, state] = solve_contact_force(contact_mod, operating_state);
```

The two local helpers contain no access to global structural objects.  Their
inputs contain only bearing-local kinematics, bearing parameters, time, and
the local contact state.  The force result is assembled by the unchanged
existing assembly path.

## Delivery phases

This commit is phase 1 only: freeze the old entry, introduce the local-contact
contract and the sole `apply_microphysics` gateway, retain transparent thermal,
roughness, and impurity modules, and prove B0/B1 identity. It must not
implement closed-loop thermal physics. The closed-loop thermal model is phase
2 and is delivered in its own later commit:

```text
feat: add transparent Stage4-B1 microphysics interface
feat: couple closed-loop thermal bearing dynamics
```

### Callback and mechanical-kernel contract

`contact_base` and `contact_mod` hold only local physical fields; neither may
hold an evaluator function handle. `operating_state` reserves the callback:

```matlab
operating_state.evaluate_raw_contact = ...
    @(contact_trial) evaluate_raw_contact(contact_trial, local, brg, t);
```

`evaluate_raw_contact` is the only mechanical Hertz-contact kernel. It must
not call `apply_microphysics`, and it returns a fixed contract for both bearing
types:

```text
result.f5, result.Q, result.delta, result.delta_raw, result.loaded,
result.loaded_count, result.element_angle, result.contact_angle,
result.normal_direction, result.contact_position, result.slice_z, result.state
```

Fields not applicable to a ball or roller bearing are empty arrays. There are
no parallel ball/roller force paths in a module: `solve_contact_force` invokes
`operating_state.evaluate_raw_contact` and derives `f5` and the public state
from its result. A later
thermal module must call the same callback and cannot infer a different result
shape from bearing type.

### Module boundaries and state contract

All three modules are transparent in phase 1. The external gateway remains:

```matlab
[contact_mod, micro_state] = apply_microphysics(...)
```

Internally every module returns its local state, for example
`[contact_mod, thermal_state] = apply_thermal_microphysics(...)`, which is
stored as `micro_state.temperature`. Roughness and impurity follow the same
state-passing pattern. The phase-1 thermal mutation allow-list excludes
`contact_stiffness`; it contains only `viscosity`, `pressure_viscosity`,
`working_clearance`, and `film_thickness`. In phase 1 all are left unchanged.

The later thermal phase will be deterministic under repeated finite-difference
calls: no `global`, `persistent`, random input, file I/O, or cross-call
temperature cache. That phase will use the callback contract, current-system
geometry/load/speed, and never fixed 20 kN/80 kN cases or their film results.

## Files

- `microphysics_config.m`: returns `thermal.enabled`, `roughness.enabled`, and
  `impurity.enabled`, all `false`.
- `apply_microphysics.m`: initializes/preserves `microphysics_state` fields
  `temperature`, `roughness_phase`, `impurity_state`, and `history`; it invokes
  enabled modules in thermal, roughness, impurity order and otherwise returns
  an identical local contact state.
- `bearing_microphysics/thermal/`: transparent placeholder constrained to
  `viscosity`, `pressure_viscosity`, `working_clearance`, and `film_thickness`.
- `bearing_microphysics/roughness/`: transparent placeholder constrained to
  `surface_height`, `effective_deformation`, `asperity_contact_ratio`,
  `contact_stiffness`, and `contact_damping`.
- `bearing_microphysics/impurity/`: transparent placeholder constrained to
  `characteristic_displacement`, `effective_deformation`, and
  `contact_stiffness`.
- `bearing_microphysics/validation/`: one B0 smoke check and one 64-step B1
  transverse regression helper; no parameter sweep or batch reporting.
- `microphysics_interface_validation.txt`: generated validation record with
  switches, numerical identity errors, call path, mutation allow-lists, and
  protected core files.

## Validation

1. Run the single Stage4-B0 zero-dynamic-load check with all switches false.
2. Run one 64-step transverse Stage4-B1 harmonic simulation on the frozen
   implementation and once on the interface implementation with all switches
   false.
3. Record `bearing_force_5dof_hist(5, nb, nt+1)` and compare complete `u`,
   `v`, and `a` histories, each bearing's `[Fx,Fy,Fz,Mx,My]` history,
   contact-body counts, local `K_b`/`C_b`, Newton iterations, and unconverged
   step counts. B0 must remain exactly zero. B1 uses
   `abs(new-frozen) <= 1e-12 + 1e-10*abs(frozen)` elementwise; only if a
   documented ordering difference remains may the relative term be relaxed to
   `1e-8`.
4. Statically scan the microphysics directory for explicit forbidden patterns:
   `newmark_newton_multi(`, `solve_static_equilibrium(`,
   `assemble_bearing_force(`, `params.modelInfo`, `num_rotor_dof`, global
   node/DOF indexing, `global`, and `persistent`. Manually confirm that it
   does not access global DOFs or M/C/K, or call Newmark, KKT, or global force
   assembly.
5. Run frozen-chain resolution under `onCleanup`; afterwards use
   `which nonlinear_bearing_force -all` to confirm that production resolves
   to the interface first and the frozen version is validation-only.
6. State that the 64-step B1 case is an interface regression, not a time-step
   convergence study.

## Explicitly protected files

`newmark_newton_multi.m`, KKT assembly/constraint logic, structural matrix
assembly, and the global force assembly contract are not edited.  The only
functional integration point is `nonlinear_bearing_force.m`.
