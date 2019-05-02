%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
clear; clc; close all;

% Cirando um Gif
nome_gif = 'Caminho_Cubica.gif';
grafico = figure;
grafico.Position = [1 41 1366 651];

% Variáveis simbolicas das juntas
syms t1 t2 l1 l2

% -----------------------------------------------------------------
% Transformações: TH = [Tz][Rz][Tx][Rx]
% -----------------------------------------------------------------
% Frame {0} Inercial
F0 = eye(4);

% Transformação do frame {0} para o frame {1}:
F1 = transformation(F0, 0, t1, 1, 0);

% Transformação do frame {1} para o frame {2}:
F2 = transformation(F1, 0, t2, 1, 0);
F2 = simplify(F2);

% ------------------------------------------------------
% Seção Gráfica
% ------------------------------------------------------
% Vetor para armazenar o trajeto do ultimo link do braço
P_path= [];

% Variação de X e Y, variáveis do caminho para determinar o movimento de cada Junta Rotacional
for x = -1:0.05:1
    y = x.^3 + 0.5;
    
    % Links do Manipulador:
    L1 = 1;  L2 = 1;
 
    C2 = ((x^2 + y^2) - (L1^2 + L2^2)) / (2*L1*L2);
    S2 = sqrt(1 - C2^2);
    
    theta2 = atan2d(S2,C2);
    
    K1 = L1 + (L2 * cosd(theta2));
    K2 = L2 * sind(theta2);
    
    r = sqrt(K1^2 + K2^2);
    
    theta1 = atan2d(y/r,x/r) - atan2d(K2,K1);
    

    % Juntas do Manipulador:
    T1 = theta1; T2 = theta2;
       
    % Parâmetros Gráficos
    plot3(0,0,0);   title(['X: ' num2str(x) 'm' '    Y' num2str(y) 'm']);
    xlabel('x'); ylabel('y'); zlabel('z');
    grid on; axis equal; hold on;
    
    % Obtendo valores numéricos para as transformações de frames:
    f0 = F0;
    f1 = double(subs(F1, [t1 l1],[T1 L1]));
    f2 = double(subs(F2, [t1 t2 l1 l2],[T1 T2 L1 L2]));
    
    % Guarda o trajeto descrito pelo End Effector
    P_path= [P_path, f2(:,4)];
    
    % ------ Manipulador ------
    % Transição do frame {0} para o {1}:
    stick_man(f0, f1)
    
    % Transição do frame {1} para o {2}:
    stick_man(f1, f2)
        
    % ------ Frames ------
    % FRAME {0}:
    % Posição da origem do frame {0}
    frame(f0, 0)
    
    % FRAME {1}:
    % Posição inicial do robô no frame {1}
    frame(f1, 1)
    
    % FRAME {2}:
    % Posição inicial do robô no frame {2}
    frame(f2, 2)
    
    plot3(P_path(1,:) , P_path(2,:) , P_path(3,:) , 'm', 'linewidth', 3)
    
    hold off;   
    drawnow;
%     gif(grafico, nome_gif);
end