%
% Sistema de controle de posicao em malha fechada
% Controlador Proporcional H(z) Kp
%
function [vetorrk,vetoruk,vetoryk,T] = IdentificacaoClosedLoopV1()
clear all;               % remove o workspace anterior
delete(timerfindall);    % deleta todos os timers anteriormente alocados
%
% parametros do controlador P
%
Kp = 0.5;
%
% parametros do tempo
% 
Fa = 10;         % Sampling frequency
T = 1/Fa;        % Sampling time
Duration = 40;   % Duracao em segundos
%%%%%
NumberOfTasksToExecute = round(Duration/T);  % <---- Numero de vezes que o controlador e' executado
%%%%%
%
% Criacao de uma onda quadrada nao simetrica de referencia com periodo Per
% Amplitude +A -> Amplitude varia de 0 Volts ate +A Volts
A   = 3.5;
Per = 10;
referencia = zeros(NumberOfTasksToExecute+1,1);
t = linspace(0,Duration,NumberOfTasksToExecute+1);
referencia = (A/2)*square(2*pi*(1/Per)*t-pi) + A/2;

% Create and configure timer object
tm = timer('ExecutionMode','fixedRate', ...            % Run continuously
    'Period',T, ...                                    % Period = sampling time
    'TasksToExecute',NumberOfTasksToExecute, ...       % Runs NumberOfTasksToExecute times
    'TimerFcn',@MyTimerFcn, ...                        % Run MyTimerFcn at each timer event
    'StopFcn',@StopEverything);
% setup da placa
s = daq.createSession('ni');
addAnalogInputChannel(s,'Dev1',0:1,'Voltage');
addAnalogOutputChannel(s,'Dev1',0,'Voltage');
s.Rate = Fa;
%
%
a1 =  1.0;
a2 =  0.0;
a3 =  0.0;
b1 =  Kp;   % Kp Ganho proporcional
b2 =  0.0;
b3 =  0.0;
% inicializacao de variaveis do sistema de controle
rk   = 0.0;
yk   = 0.0;
%
uk   = 0.0;
uk_1 = 0.0;
uk_2 = 0.0;
%
ek   = 0.0;
ek_1 = 0.0;
ek_2 = 0.0;
%
% Inicializacao do vetor que guarda o historico das variaveis
%
vetorrk = zeros(NumberOfTasksToExecute+1,1);
vetorrk(1) = 0; % instante 0 -> k=1
vetorrk(2) = 0;
vetoryk = zeros(NumberOfTasksToExecute+1,1);
vetoryk(1) = 0;
vetoryk(2) = 0;
vetoruk = zeros(NumberOfTasksToExecute+1,1);
vetoruk(1) = 0;
vetoruk(2) = 0;
% inicializacao da variavel do tempo discreto
k = 3;
% Start the timer
start(tm);
%
% Funcao que e' executada no timer tm periodicamente a cada T    <--------------------------------------
%
function MyTimerFcn(~,~)    
% Leitura da Porta A/D
sample = inputSingleScan(s);     % le o dado do canal de entrada 0, 1    
rk = referencia(k);
% Escolher aqui ou velocidade angular ou posicao angular
yk = sample(2);               % leitura da tensao do potenciometro                  
%
% calculo do erro e(k)
%
ek = rk - yk;    
% Calculo do controle
uk = (-a2/a1)*uk_1+(-a3/a1)*uk_2+(b1/a1)*ek+(b2/a1)*ek_1+(b3/a1)*ek_2;
% Controle de Saturacao
if uk >= 5.0 
   uk = 5.0;
else if uk <= -5.0
   uk = -5.0;
    end
end % fim da saturacao
% Escrita na Porta D/A - u(k)
outputSingleScan(s,uk);
% Salva os valores
vetorrk(k) = rk;
vetoryk(k) = yk;
vetoruk(k) = uk;
% Update de variaveis    
k = k+1;
uk_2 = uk_1; 
uk_1 = uk;
ek_2 = ek_1;   
ek_1 = ek;
end % Fim da funcao MyTymerFcn
% Funcao executada ao final da execucao do timer tm
function StopEverything(~,~)
    outputSingleScan(s,0.0);
    release(s);
    nsamples = length(vetorrk);
    t=0.0:T:(nsamples-1)*T;
    plot(t,vetorrk,'-',t,vetoryk,'--');
    hold on
    stairs(t,vetoruk);
    grid on
    title('Referencia r(k) / Esforco de controle u(k) /    Saida da planta y(k)')
    xlabel('tempo (s)');
    ylabel('tensao (Volts)');
    
    %save('lixo3') % apenas usado para salvar o workspace qdo necessario
    
    matdata = [t' vetorrk vetoruk vetoryk]; % agrupa variaveis de interesse 
        
    % coloque aqui o nome do arquivo onde deseja salvar os dados
    save('filename3.txt','matdata','-ascii')
    
    mensagem = 'acabou'    
end % Fim de StopEverything

end  % FIM function controlador
