function test_microphysics_regression
%TEST_MICROPHYSICS_REGRESSION Acceptance test for the B0/B1 interface record.

report_file = fullfile(fileparts(mfilename('fullpath')), '..', '..', ...
    'microphysics_interface_validation.txt');
before = dir(report_file);
result = run_microphysics_interface_validation(false);
after = dir(report_file);
assert(result.b0.pass, 'B0 must retain its strict zero-dynamic-load acceptance.');
assert(result.b1.pass, 'B1 interface regression exceeded its mixed tolerance.');
assert(result.switches.thermal == false && result.switches.roughness == false && ...
    result.switches.impurity == false, 'All microphysics switches must remain disabled.');
assert(strcmp(result.source_branch, 'brunch-2-micro'), ...
    'The frozen source branch must be recorded in the validation result.');
assert(strcmp(result.source_commit, '1afcebecef812481dc05a2f054e5a3766d0cbf70'), ...
    'The frozen source commit must be recorded in the validation result.');
assert(isequaln(before.datenum, after.datenum), ...
    'write_report=false must not update the validation report file.');
end
