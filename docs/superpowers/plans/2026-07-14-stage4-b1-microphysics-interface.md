# Stage4-B1 Microphysics Interface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a transparent, local-only microphysics interface while preserving the frozen Stage4-B1 result when every switch is disabled.

**Architecture:** `nonlinear_bearing_force.m` constructs a bearing-local base contact record, invokes the one microphysics gateway, then sends the returned record through the original local force law.  The gateway owns state initialization and whitelist checks; transparent thermal, roughness, and impurity placeholders return no numerical changes.

**Tech Stack:** MATLAB R2024a, existing Stage4A/B Newmark validation runners, Git.

## Global Constraints

- Work from `brunch-2-micro` and preserve a frozen pre-interface implementation.
- Do not change Newmark variables, KKT logic, global 192-DOF assembly, or global M/C/K matrices.
- `thermal.enabled`, `roughness.enabled`, and `impurity.enabled` default to `false`.
- No physical microphysics model and no batch result study.
- Validate one B0 check, then one 64-step Stage4-B1 transverse harmonic regression.
- Pass only local contact data to microphysics modules.

---

### Task 1: Freeze the Stage4-B1 contact entry and write a failing interface test

**Files:**
- Create: `stage4_5dof_bearing_backup/nonlinear_bearing_force_stage4B1_frozen.m`
- Create: `bearing_microphysics/validation/test_microphysics_interface.m`

**Interfaces:**
- Consumes: current `nonlinear_bearing_force(q, qd, params, bearing, MM_size)` behavior.
- Produces: a frozen baseline callable and a test that requires `microphysics_config` and `apply_microphysics`.

- [ ] **Step 1: Copy the current entry into the named frozen baseline file and rename its top-level function.**
- [ ] **Step 2: Write the failing test requiring all switches false, initialized state fields, and an unchanged contact struct.**

```matlab
cfg = microphysics_config();
base = struct('contact_stiffness', 1, 'working_clearance', 2);
[actual, state] = apply_microphysics(base, struct(), struct(), cfg);
assert(isequaln(actual, base));
assert(all(isfield(state, {'temperature','roughness_phase','impurity_state','history'})));
```

- [ ] **Step 3: Run the test and observe failure because `microphysics_config` is undefined.**
- [ ] **Step 4: Commit the frozen entry and red test.**

### Task 2: Implement the transparent configuration, gateway, and placeholders

**Files:**
- Create: `microphysics_config.m`
- Create: `apply_microphysics.m`
- Create: `bearing_microphysics/thermal/apply_thermal_microphysics.m`
- Create: `bearing_microphysics/roughness/apply_roughness_microphysics.m`
- Create: `bearing_microphysics/impurity/apply_impurity_microphysics.m`
- Modify: `bearing_microphysics/validation/test_microphysics_interface.m`

**Interfaces:**
- Consumes: `contact_base`, `operating_state`, `micro_state`, `cfg`.
- Produces: `[contact_mod, micro_state] = apply_microphysics(contact_base, operating_state, micro_state, cfg)`.

- [ ] **Step 1: Implement `microphysics_config` with three false switches and module directories.**
- [ ] **Step 2: Implement the gateway with strict field whitelists and deterministic state initialization.**
- [ ] **Step 3: Implement transparent modules that preserve the contact record exactly and append no numerical model corrections.**
- [ ] **Step 4: Run `test_microphysics_interface`; expect pass.**
- [ ] **Step 5: Commit the gateway and placeholders.**

### Task 3: Integrate the sole local gateway without changing structural assembly

**Files:**
- Modify: `nonlinear_bearing_force.m`
- Modify: `bearing_microphysics/validation/test_microphysics_interface.m`

**Interfaces:**
- Consumes: existing Stage4A local kinematics and bearing parameters.
- Produces: `build_base_contact_state`, `solve_contact_force`, and a single `apply_microphysics` invocation on each local contact evaluation.

- [ ] **Step 1: Extend the failing test to require that `nonlinear_bearing_force.m` contains one gateway invocation and no microphysics module references to Newmark/KKT/M/C/K.**
- [ ] **Step 2: Verify the structural scan fails before integration.**
- [ ] **Step 3: Extract the existing Stage4A ball/roller local calculation into side-effect-free contact-record construction and force solution helpers.**
- [ ] **Step 4: Add the single gateway call between those helpers, forwarding only local state and `params.microphysics` configuration.**
- [ ] **Step 5: Run the unit test and MATLAB `checkcode` for all created/modified MATLAB files.**
- [ ] **Step 6: Commit the integration.**

### Task 4: Run B0 then a 64-step transverse Stage4-B1 identity regression and write the record

**Files:**
- Create: `bearing_microphysics/validation/run_microphysics_interface_validation.m`
- Create: `microphysics_interface_validation.txt`

**Interfaces:**
- Consumes: frozen baseline, interface implementation, `initial_conditions`, `newmark_newton_multi`.
- Produces: validation record with switches, B0 status, B1 full-history errors, call path, allow-lists, and protected files.

- [ ] **Step 1: Write a failing validation test that calls B0 first and requires zero maximum error for the 64-step transverse B1 displacement and bearing-force histories.**
- [ ] **Step 2: Run it and observe failure because the validator does not exist.**
- [ ] **Step 3: Implement B0 followed by exactly one frozen and one interface 64-step B1 transverse simulation.**
- [ ] **Step 4: Generate `microphysics_interface_validation.txt` from measured values and static boundary checks.**
- [ ] **Step 5: Run the complete validator and inspect the generated report.**
- [ ] **Step 6: Commit all interface code, validator, report, and test.**

### Task 5: Final verification and baseline-version commit

**Files:**
- Verify: all files listed above.

- [ ] **Step 1: Run `checkcode` across each changed MATLAB file.**
- [ ] **Step 2: Run the complete microphysics interface validation from a clean MATLAB process.**
- [ ] **Step 3: Inspect `git diff --check`, `git status`, and confirm protected files are unmodified.**
- [ ] **Step 4: Commit with message `feat: add microphysics interface baseline`.**
