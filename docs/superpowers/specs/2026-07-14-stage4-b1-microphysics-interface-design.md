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
  regression comparison.

## Architecture

`nonlinear_bearing_force.m` becomes an orchestration boundary.  Per bearing it
creates a local `contact_base`, calls the sole microphysics entry, then solves
the force from the returned local state:

```matlab
contact_base = build_base_contact_state(...);
[contact_mod, micro_state] = apply_microphysics( ...
    contact_base, operating_state, micro_state, cfg);
[f5, state] = solve_contact_force(contact_mod);
```

The two local helpers contain no access to global structural objects.  Their
inputs contain only bearing-local kinematics, bearing parameters, time, and
the local contact state.  The force result is assembled by the unchanged
existing assembly path.

## Files

- `microphysics_config.m`: returns `thermal.enabled`, `roughness.enabled`, and
  `impurity.enabled`, all `false`.
- `apply_microphysics.m`: initializes/preserves `microphysics_state` fields
  `temperature`, `roughness_phase`, `impurity_state`, and `history`; it invokes
  enabled modules in thermal, roughness, impurity order and otherwise returns
  an identical local contact state.
- `bearing_microphysics/thermal/`: transparent placeholder constrained to
  `viscosity`, `pressure_viscosity`, `working_clearance`, `film_thickness`, and
  `contact_stiffness`.
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
3. Compare complete displacement and bearing-force histories by maximum
   absolute error.  The acceptance threshold is exactly zero, excluding only
   fields intentionally added for microphysics state reporting.
4. Staticaly check that no module references Newmark, KKT, 192-DOF, or global
   M/C/K symbols.

## Explicitly protected files

`newmark_newton_multi.m`, KKT assembly/constraint logic, structural matrix
assembly, and the global force assembly contract are not edited.  The only
functional integration point is `nonlinear_bearing_force.m`.
