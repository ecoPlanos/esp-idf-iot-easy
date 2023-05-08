sen_ref_values=[-40 -25 0 25 85 100 180];
sen_ref_resists=[333562 129925 32639 10000 1070 678.1 96.07];
sen_ref_res_tols=[7.8 7.04 5.94 5 6.77 7.14 8.72];
temp_tols=[1.18 1.18 1.16 1.14 2.15 2.44 4.29];
sen_unit='℃';
div_res=2500;
div_res_tol=0.05;
div_res_tol=1;
adc_bit=12;
max_v=3.3;
poly_n=1;
tune_idx=[1:length(sen_ref_values)];
tune_idx=[4:length(sen_ref_values)-1];

poly_exp=poly_n:-1:0;
resol_mv=max_v/2^adc_bit;

v_temps=(max_v*div_res)./(div_res+sen_ref_resists);
v_temps_max=(max_v*(div_res+div_res*(div_res_tol/100)))./(div_res+div_res*(div_res_tol/100)+sen_ref_resists-sen_ref_resists.*(sen_ref_res_tols/100));
v_temps_min=(max_v*(div_res-div_res*(div_res_tol/100)))./(div_res-div_res*(div_res_tol/100)+sen_ref_resists+sen_ref_resists.*(sen_ref_res_tols/100));

p=polyfit(v_temps(tune_idx),sen_ref_values(tune_idx),poly_n);
calc_temps=sum(v_temps.^repmat(poly_exp',1,length(v_temps)).*repmat(p',1,length(v_temps)));
calc_error=abs(abs(sen_ref_values)-abs(calc_temps));

calc_temps_min=sum(v_temps_min.^repmat(poly_exp',1,length(v_temps)).*repmat(p',1,length(v_temps)));
calc_temps_max=sum(v_temps_max.^repmat(poly_exp',1,length(v_temps)).*repmat(p',1,length(v_temps)));
res_min_error=abs(abs(calc_temps)-abs(calc_temps_max));
res_max_error=abs(abs(calc_temps)-abs(calc_temps_min));
total_min_error=abs(calc_temps_min)+res_min_error;
total_max_error=abs(calc_temps)+res_max_error;
min_res_error=min([res_min_error(tune_idx),res_max_error(tune_idx)]);
max_res_error=max([res_min_error(tune_idx),res_max_error(tune_idx)]);

disp(['reference resistance: ',num2str(div_res),' Ω'])
disp(['maximum calculation error: ',num2str(max(calc_error(tune_idx))),' ',sen_unit])
disp(['minimum calculation error: ',num2str(min(calc_error(tune_idx))),' ',sen_unit])
disp(['maximum sensor error: ',num2str(max_res_error),' ',sen_unit])
disp(['minimum sensor error: ',num2str(min_res_error),' ',sen_unit])
disp(['maximum error: ',num2str(max(calc_error(tune_idx))+max_res_error),' ',sen_unit])
disp(['minimum error: ',num2str(min(calc_error(tune_idx))+min_res_error),' ',sen_unit])
disp(['maximum voltage output: ',num2str(max(v_temps(tune_idx))),' V'])
disp(['min voltage output: ',num2str(min(v_temps(tune_idx))),' V'])
disp(['output voltage range: ',num2str(max(v_temps(tune_idx))-min(v_temps(tune_idx))),' V'])
disp(['sensor max current: ',num2str(max(v_temps(tune_idx)./(sen_ref_resists(tune_idx)-sen_ref_resists(tune_idx).*(sen_ref_res_tols(tune_idx)/100)+div_res-div_res*(div_res_tol/100)))*1000),' mA'])
disp(['sensor max dissipated power: ',num2str(max((v_temps(tune_idx).^2)./(sen_ref_resists(tune_idx)-sen_ref_resists(tune_idx).*(sen_ref_res_tols(tune_idx)/100)+div_res-div_res*(div_res_tol/100)))*1000),' mW'])
poly_str='f(x) = ';
for i=length(poly_exp):-1:1
    poly_str=[poly_str,num2str(p(i)),'*x^',num2str(poly_exp(i))];
    if i>1
        poly_str=[poly_str, ' + '];
    end
end
disp(['resulting poly: ',poly_str]);
figure()
plot(v_temps,[sen_ref_values' calc_temps']); grid on;
title('convertion vs reference')
xlabel('Voltage (V)')
ylabel('temperature (℃)')
legend('Reference','Calculated')
