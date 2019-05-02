%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
clear; clc; close all;

% Cirando um Gif
nome_gif = 'Manipuladores_Trabalho_2_RPP.gif';
grafico = figure;
grafico.Position = [1 41 1366 651];

% Variáveis simbolicas das juntas
syms t1 t2 t3 t4 l1 l2 l3 l4

% -----------------------------------------------------------------
% Transformações: TH = [Tz][Rz][Tx][Rx]
% -----------------------------------------------------------------
% Frame {0} Inercial
F0 = eye(4);

% Transformação do frame {0} para o frame {1}:
F1 = transformation(F0, 3, t1, 0, 0);

% Transformação do frame {1} para o frame {2}:
F2 = transformation(F1, l2, 0, 0, -90);

% Transformação do frame {2} para o frame {3}:
F3 = transformation(F2, l3, 0, 0, 0);

% ------------------------------------------------------
% Seção Gráfica
% ------------------------------------------------------
% Vetor para armazenar o trajeto do ultimo link do braço
P_path= [];

% Controle da junta prismática
link = 0;
flag  = 1;

% Variação do angulo Teta de cada Junta Rotcional
for theta = 0:360
    
    % Parâmetros Gráficos
    plot3(0,0,0);   title(['Angulo: ' num2str(theta) '°']);
    xlabel('x'); ylabel('y'); zlabel('z');
    grid on; axis equal; hold on;
    
    % Juntas do Manipulador:
    T1 = theta; T2 = theta; T3 = theta; T4 = theta;
    L1 = link;  L2 = link;  L3 = link;  L4 = link;
    
    % Variação da translação L de cada Junta Prismática
    if(link <= 3 && flag == 1)
        link = link + 1;
    end
    
    if(link >= 0 && flag == 0)
        link = link - 1;
    end
    
    if(link == 3)
        flag = 0;
    end
    
    if(link == 0)
        flag = 1;
    end
    
    % Obtendo valores numéricos para as transformações de frames:
    f0 = F0;
    f1 = double(subs(F1, [t1 l1],[T1 L1]));
    f2 = double(subs(F2, [t1 t2 l1 l2],[T1 T2 L1 L2]));
    f3 = double(subs(F3, [t1 t2 t3 l1 l2 l3],[T1 T2 T3 L1 L2 L3]));
    
    % Guarda o trajeto descrito pelo End Effector
    P_path= [P_path, f3(:,4)];
    
    % ------ Manipulador ------
    % Transição do frame {0} para o {1}:
    stick_man(f0, f1)
    
    % Transição do frame {1} para o {2}:
    stick_man(f1, f2)
    
    % Transição do frame {2} para o {3}:
    stick_man(f2, f3)
    
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
    
    % FRAME {3}:
    % Posição inicial do robô no frame {3}
    frame(f3, 3)
    
    plot3(P_path(1,:) , P_path(2,:) , P_path(3,:) , 'b')
    
    hold off;   drawnow;
%     gif(grafico, nome_gif);
pause;
end

