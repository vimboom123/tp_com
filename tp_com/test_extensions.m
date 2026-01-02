% 测试脚本：验证 Extensions 文件的 BER 结果
% 只测试 Doppler sweep 中的 fdTs=0（block fading baseline）
run_rice_sweep = false;
run_corr_sweep = false;
run_doppler_sweep = true;
fast_mode = false;
doppler_tracking = false;

% 运行脚本
Main_Alamouti_Extensions

% 打印详细结果
fprintf('\n=== Doppler Sweep BER Results ===\n');
for id = 1:size(BER_Doppler, 1)
    fprintf('fdTs=%.3f: ', fdTs_list(id));
    for k = 1:length(Lsnr_dB)
        fprintf('%.2e ', BER_Doppler(id, k));
    end
    fprintf('\n');
end
