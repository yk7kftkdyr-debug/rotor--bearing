# Closed-Loop Thermal Bearing Dynamics Design

## Scope

Implement the second, local-only thermal coupling commit on `brunch-2-micro`. The thermal module is entered only through `apply_microphysics`; it uses the existing `operating_state.evaluate_raw_contact` callback and never accesses Newmark, KKT, global DOFs, M/C/K, or global force assembly.

## Raw-contact invariant

With `thermal.enabled=false`, `evaluate_raw_contact` follows the existing Stage4A mechanical calculation exactly. Its working clearance is each bearing's assembly radial clearance and its film-thickness contribution is zero. B0 and frozen B1 therefore remain exact identity regressions.

With thermal enabled, the raw-contact kernel consumes only:
- scalar `working_clearance`, with the existing Stage4A clearance sign convention;
- `film_thickness`, expanded to the exact contact grid of `Q/delta` (ball: 1-by-n; roller: n-by-ns).

An empty or scalar zero film field is normalized to an all-zero contact-grid array. A non-scalar film must match the contact grid exactly. The effective deformation replaces the original clearance term exactly once: geometry minus working clearance minus film thickness. No other code subtracts clearance or film; `K_point`, `K_line`, final force, and global tangents are never scaled.

## Local closed loop

Each call starts from oil temperature, computes temperature-dependent viscosity, pressure-viscosity, clearance, and current-system film estimate, evaluates the raw contact callback, computes source-derived local drag/slip power, and relaxes temperature. It has no cross-call state. At convergence it fully repeats the property, film, raw-contact, and diagnostics sequence at `T_final`, returning only that result.

## Verification

Run thermal-disabled B0 and frozen B1 before any enabled check. Verify 20/50/80/100 C properties, both bearing local equilibria, then paired 64-step B1 runs. The sole generated output is `thermal_coupling_validation.txt`; protected baseline code and `microphysics_interface_validation.txt` remain unchanged.

