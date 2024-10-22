function plot_voltage_time_input(u, t, name)
%PLOT_VOLTAGE_CURRENT Plot Voltage x Current
%   Makes a Voltage X Current plot from the simulation output.
%   The image will be saved automatically if the folder is argument 
%   is given
    
    name = strcat(name, ' - Esforço de Controle');

    figure;
    
    pl = plot(t, u);
    [~, index] = max(abs(u));
    str = {'Tempo: ', 'Tensão de Pico: '};
    datatip(pl, index, str, 'hold');
    
    ylabel('Esforço de Controle [V]');
    xlabel('Tempo [s]');
    title(name);
    
    name = removeSpecialCharacters(name);
    saveas(gcf, strcat('imagens/', name, '.png'));
end

