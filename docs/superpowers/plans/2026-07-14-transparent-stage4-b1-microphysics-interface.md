# Transparent Stage4-B1 Microphysics Interface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a transparent local-only Stage4-B1 microphysics interface that exactly preserves the frozen force chain while all modules are disabled.

**Architecture:** `nonlinear_bearing_force` creates local physical state and an `operating_state` callback to one raw mechanical contact kernel. `apply_microphysics` passes module state without changing phase-1 physics. `solve_contact_force` invokes the same callback and maps its result into the existing state and assembly path.

**Tech Stack:** MATLAB R2024a, existing Stage4A/B 5-DOF contact law, Newmark validation harness, Git.

## Global Constraints

- Work only on `brunch-2-micro`.
- Phase 1 only: no closed-loop thermal physics; all three module switches remain false.
- Do not modify Newmark, KKT, `build_rotor_case_model.m`, global M/C/K, `assemble_bearing_force.m`, 192-DOF definition, or frozen files.
- Contact records contain physical fields only. The raw callback exists only in `operating_state`.
- The phase-1 thermal allow-list is only `viscosity`, `pressure_viscosity`, `working_clearance`, and `film_thickness`; all remain unchanged.
- No figures, Office/CSV/MAT artifacts, temporary outputs, or output files other than `microphysics_interface_validation.txt`.

---

### Task 1: Establish one raw mechanical contact contract

**Files:**
- Modify: `nonlinear_bearing_force.m`
- Modify: `bearing_microphysics/validation/run_microphysics_interface_validation.m`

**Interfaces:**
- Produces: `result = evaluate_raw_contact(contact_trial, local, brg, t)`.
- Produces: `[f5,state] = solve_contact_force(contact_mod, operating_state)`.

- [ ] **Step 1: Write the failing contract check**

Add a validation helper requiring the raw result fields:

```matlab
required = {'f5','Q','delta','delta_raw','loaded','loaded_count', ...
    'element_angle','contact_angle','normal_direction', ...
    'contact_position','slice_z','state'};
assert(all(isfield(result, required)), 'Raw contact contract is incomplete.');
```

- [ ] **Step 2: Run the validation and verify failure**

Run:

```bash
/Applications/MATLAB_R2024a.app/bin/matlab -batch "cd('/Users/bailingguan/Documents/Codex/2026-07-14/di/work/rotor-bearing'); run_microphysics_interface_validation"
```

Expected: FAIL because no type-independent raw contact result exists.

- [ ] **Step 3: Implement the one mechanical kernel**

In `nonlinear_bearing_force.m`, introduce:

```matlab
function result = evaluate_raw_contact(contact_trial, local, brg, t)
result = struct('f5', zeros(5,1), 'Q', [], 'delta', [], 'delta_raw', [], ...
    'loaded', [], 'loaded_count', 0, 'element_angle', [], ...
    'contact_angle', [], 'normal_direction', [], 'contact_position', [], ...
    'slice_z', [], 'state', struct());
if strcmpi(brg.type, 'ball')
    result = raw_ball_contact(result, contact_trial, local, brg, t);
else
    result = raw_roller_contact(result, contact_trial, local, brg, t);
end
end
```

Move the existing Hertz geometry, exponents, and force signs into `raw_ball_contact` and `raw_roller_contact` unchanged. Fill every result field; use empty arrays for fields inapplicable to the bearing type.

- [ ] **Step 4: Make force extraction use only the callback**

Implement:

```matlab
function [f5, state] = solve_contact_force(contact_mod, operating_state)
result = operating_state.evaluate_raw_contact(contact_mod);
f5 = result.f5;
state = result.state;
end
```

Do not perform Hertz calculations in `solve_contact_force`.

- [ ] **Step 5: Run the contract check and commit**

Run the Step 2 command. Expected: contract check PASS.

```bash
git add nonlinear_bearing_force.m bearing_microphysics/validation/run_microphysics_interface_validation.m
git commit -m "refactor: unify Stage4A raw contact kernel"
```

### Task 2: Make the module gateway transparently return state

**Files:**
- Modify: `apply_microphysics.m`
- Modify: `microphysics_config.m`
- Modify: `bearing_microphysics/thermal/apply_thermal_microphysics.m`
- Modify: `bearing_microphysics/roughness/apply_roughness_microphysics.m`
- Modify: `bearing_microphysics/impurity/apply_impurity_microphysics.m`

**Interfaces:**
- External: `[contact_mod,micro_state] = apply_microphysics(contact_base,operating_state,micro_state,cfg)`.
- Internal: every module returns `[contact_mod,module_state]`.

- [ ] **Step 1: Write the failing disabled-gateway test**

Add:

```matlab
assert(isequaln(contact_mod, contact_base), 'Disabled module changed contact physics.');
assert(all(isfield(micro_state, {'temperature','roughness_phase','impurity_state'})), ...
    'Microphysics state slots are incomplete.');
```

- [ ] **Step 2: Run it to verify failure**

Run the Task 1 MATLAB command. Expected: FAIL due to missing module-state return handoff.

- [ ] **Step 3: Implement state-returning transparent modules**

Implement all three transparent modules with their own state slot:

```matlab
function [contact_mod, thermal_state] = apply_thermal_microphysics(contact_mod, ~, ~, ~)
thermal_state = struct('enabled', false, 'mode', 'transparent');
end
```

```matlab
function [contact_mod, roughness_state] = apply_roughness_microphysics(contact_mod, ~, ~, ~)
roughness_state = struct('enabled', false, 'mode', 'transparent');
end

function [contact_mod, impurity_state] = apply_impurity_microphysics(contact_mod, ~, ~, ~)
impurity_state = struct('enabled', false, 'mode', 'transparent');
end
```

Update `run_module` to return `[output,module_state]`; assign it to `micro_state.temperature`, `micro_state.roughness_phase`, or `micro_state.impurity_state`. Retain the allow-list assertion and set thermal allowed fields to exactly:

```matlab
{'viscosity','pressure_viscosity','working_clearance','film_thickness'}
```

Keep every default switch false.

- [ ] **Step 4: Run it to verify pass and commit**

Run the Task 1 MATLAB command. Expected: PASS and disabled contact records unchanged.

```bash
git add apply_microphysics.m microphysics_config.m bearing_microphysics
git commit -m "feat: preserve transparent microphysics module state"
```

### Task 3: Integrate the callback only through operating state

**Files:**
- Modify: `nonlinear_bearing_force.m`

**Interfaces:**
- Consumes: current local state, bearing record, and time.
- Produces: `operating_state.evaluate_raw_contact`; unchanged global assembly inputs.

- [ ] **Step 1: Write the failing physical-record check**

Before gateway invocation add:

```matlab
assert(~any(structfun(@(value) isa(value, 'function_handle'), contact_base)), ...
    'contact_base must not contain a callback.');
```

- [ ] **Step 2: Run it to verify failure**

Run the Task 1 MATLAB command. Expected: FAIL until the callback is supplied and consumed through `operating_state`.

- [ ] **Step 3: Integrate the callback**

Use:

```matlab
operating_state = struct('time', t, 'bearing_index', ib);
operating_state.evaluate_raw_contact = ...
    @(contact_trial) evaluate_raw_contact(contact_trial, local, brg, t);
[contact_mod, micro_state] = apply_microphysics( ...
    contact_base, operating_state, struct(), micro_cfg);
[f5, state] = solve_contact_force(contact_mod, operating_state);
```

Do not add a callback to contact records or change `assemble_bearing_force`.

- [ ] **Step 4: Run B0/B1 identity and commit**

Run the Task 1 MATLAB command. Expected: strict B0 zero and PASS for complete frozen/new B1 identity.

```bash
git add nonlinear_bearing_force.m
git commit -m "feat: add transparent Stage4-B1 microphysics interface"
```

### Task 4: Record recoverable frozen-path and full 5-DOF evidence

**Files:**
- Modify: `bearing_microphysics/validation/run_microphysics_interface_validation.m`
- Modify: `microphysics_interface_validation.txt`

**Interfaces:**
- Produces: `bearing_force_5dof_hist(5,nb,nt+1)` and restored-path evidence.

- [ ] **Step 1: Write failing five-DOF/path assertions**

Add:

```matlab
history = zeros(5, nb, size(sim.Fb_global_hist, 2));
assert(isequal(size(history), [5 nb size(sim.Fb_global_hist,2)]));
paths = which('nonlinear_bearing_force', '-all');
assert(strcmp(paths{1}, fullfile(root, 'nonlinear_bearing_force.m')), ...
    'Frozen validation path leaked into production resolution.');
```

- [ ] **Step 2: Run it to verify failure**

Run the Task 1 MATLAB command. Expected: FAIL because the current comparison is flattened and does not prove restored production resolution.

- [ ] **Step 3: Implement full history, recoverable path, and precise boundary scan**

Build history as:

```matlab
for ib = 1:nb
    index = 6 * params.bearing(ib).rotor_node + (-5:-1);
    history(:,ib,:) = reshape(sim.Fb_global_hist(index,:), 5, 1, []);
end
```

Use `onCleanup(@() restore_interface_path(...))` for the frozen shim. After cleanup, call `which('nonlinear_bearing_force','-all')`. Scan module files only for:

```matlab
patterns = {'newmark_newton_multi\\(', 'solve_static_equilibrium\\(', ...
    'assemble_bearing_force\\(', 'params\\.modelInfo', 'num_rotor_dof', ...
    '\\<global\\>', '\\<persistent\\>'};
```

Reject a match and manually verify no global DOF/node indexing, M/C/K, KKT, Newmark, or global assembly access.

- [ ] **Step 4: Run final validation and commit**

Run the Task 1 MATLAB command. Expected: PASS with B0 zeros; B1 comparisons for `u/v/a`, both five-DOF histories, loaded count, `Kb/Cb`, Newton counts, and unconverged steps all pass.

```bash
git add bearing_microphysics/validation/run_microphysics_interface_validation.m microphysics_interface_validation.txt
git commit -m "test: verify transparent microphysics interface regression"
```

### Task 5: Audit protected boundaries and artifact scope

**Files:**
- Modify: none unless Task 4 finds an issue.

- [ ] **Step 1: Confirm protected files are unmodified**

Run:

```bash
git diff origin/brunch-2-micro -- newmark_newton_multi.m build_rotor_case_model.m assemble_bearing_force.m bearing_microphysics/validation/frozen_chain/nonlinear_bearing_force_stage4B1_frozen.m
```

Expected: no output.

- [ ] **Step 2: Confirm branch and artifact scope**

Run:

```bash
git status --short
git diff --name-only origin/brunch-2-micro
```

Expected: no untracked artifacts; only required source, validation code, validation record, and approved design/plan documents.

- [ ] **Step 3: Ensure the required implementation commit title exists**

Run:

```bash
git log --oneline --all --grep='feat: add transparent Stage4-B1 microphysics interface' -1
```

Expected: one implementation commit.
