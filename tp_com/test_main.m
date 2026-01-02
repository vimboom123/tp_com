% 测试主文件 baseline
Main_Alamouti_Full_Project;
fprintf('\n=== Main File BER Results ===\n');
fprintf('SNR: '); fprintf('%d ', Lsnr_dB); fprintf('\n');
fprintf('SISO: '); fprintf('%.2e ', BER_SISO); fprintf('\n');
fprintf('Alamouti: '); fprintf('%.2e ', BER_Alamouti); fprintf('\n');
