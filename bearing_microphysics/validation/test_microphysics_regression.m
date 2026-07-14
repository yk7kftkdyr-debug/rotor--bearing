function test_microphysics_regression
%TEST_MICROPHYSICS_REGRESSION Acceptance test for the B0/B1 interface record.

result = run_microphysics_interface_validation();
assert(result.b0.pass, 'B0 must retain its strict zero-dynamic-load acceptance.');
assert(result.b1.pass, 'B1 interface regression exceeded its mixed tolerance.');
assert(result.switches.thermal == false && result.switches.roughness == false && ...
    result.switches.impurity == false, 'All microphysics switches must remain disabled.');
end
