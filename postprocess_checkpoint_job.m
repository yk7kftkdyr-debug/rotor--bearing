function postprocess_checkpoint_job(checkpoint_file)
%POSTPROCESS_CHECKPOINT_JOB Generate figures/reports from a saved solver state.
% This helper keeps long Newmark time integration and MATLAB graphics export
% in separate processes. It does not change any mechanical result.

S = load(checkpoint_file, 'params', 'bearingQD', 'sim');
params = S.params;
bearingQD = S.bearingQD;
sim = S.sim;

if ~isfolder(params.output_dir)
    mkdir(params.output_dir);
end
if ~isfolder(params.figure_dir)
    mkdir(params.figure_dir);
end

close all force;
post = post_process(sim, params, bearingQD);
report_generator(params, bearingQD, sim, post);
generate_word_manual(params.manual_file);
save(params.result_mat_file, 'params', 'bearingQD', 'sim', 'post', '-v7.3');
close all force;
fprintf('Postprocess completed: %s\n', params.result_mat_file);
end
