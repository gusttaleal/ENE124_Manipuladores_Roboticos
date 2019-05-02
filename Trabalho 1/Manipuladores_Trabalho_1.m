%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
clear; clc; close all;

% Cirando um Gif
% nome_gif = 'Manipulador_Trabalho1_6.gif';
% grafico = figure;
% grafico.Position = [1 41 1366 651];

% Transformações Homogêneas para modelagem de robô manipuladore com 7 DoF
% Variáveis simbólicas para os parâmetros: Ângulo Teta [th], Delta X [dx], Delta Y [dy], Delta Z [dz]
syms th dx dy dz 

% Variáveis simbólicas para os parâmetros variáveis: Ângulo Teta N [th1 ... th7]
syms th1 th2 th3 th4 th5 th6 th7

% Variáveis não simbólicas para os parâmetros fixos:
BS = 0.5;   % Base
L1 = 1.1;   % Link 1
L2 = 1.1;   % Link 2
L3 = 1.0;   % Link 3
L4 = 1.0;   % Link 4
L5 = 0.8;   % Link 5
L6 = 0.8;   % Link 6

% Matriz de translação (generalista)
T = [1 0 0 dx; 0 1 0 dy; 0 0 1 dz; 0 0 0 1];
% Matriz de rotação em x (generalista)
Rx = [1 0 0 0; 0 cos(th) -sin(th) 0; 0 sin(th) cos(th) 0; 0 0 0 1];
% Matriz de rotação em y (generalista)
Ry = [cos(th) 0 sin(th) 0; 0 1 0 0; -sin(th) 0 cos(th) 0; 0 0 0 1];
% Matriz de rotação em z (generalista)
Rz = [cos(th) -sin(th) 0 0; sin(th) cos(th) 0 0; 0 0 1 0; 0 0 0 1];

% Frame {0} Inercial
F0 = eye(4);

% ------------------------------------------------------
% Transformação do frame Inercial para o frame {1}: TH = [T][Rx][Rz]
% ------------------------------------------------------
TH1 = subs(T, [dx dy dz], [0 0 BS]) * subs(Rx, th, 0) * subs(Rz, th, 0);
% Transformação para dar liberdade a junta
Rz1 = subs(Rz, th, deg2rad(th1));
% Frame {1}
F1 = (F0 * TH1 * Rz1);

% ------------------------------------------------------
% Transformação do frame Inercial para o frame {2}: TH = [T][Rz][Rx]
% ------------------------------------------------------
TH2 = (subs(T, [dx dy dz], [0 0 L1]) * subs(Rz, th, deg2rad(90)) * subs(Rx, th, deg2rad(-90)));
% Transformação para dar liberdade a junta
Rz2 = subs(Rz, th, deg2rad(th2));
% Frame {2}
F2 = (F1 * TH2 * Rz2);

% ------------------------------------------------------
% Transformação do frame Inercial para o frame {3}: TH = [Rx][Rz][T]
% ------------------------------------------------------
TH3 = (subs(Rx, th, deg2rad(90)) * subs(Rz, th, deg2rad(-90)) * subs(T, [dx dy dz], [0 0 L2]));
% Transformação para dar liberdade a junta
Rz3 = subs(Rz, th, deg2rad(th3));
% Frame {3}
F3 = (F2 * TH3 * Rz3);

% ------------------------------------------------------
% Transformação do frame Inercial para o frame {4}: TH = [T][Rz][Rx]
% ------------------------------------------------------
TH4 = (subs(T, [dx dy dz], [0 0 L3]) * subs(Rz, th, deg2rad(90)) * subs(Rx, th, deg2rad(-90)));
% Transformação para dar liberdade a junta
Rz4 = subs(Rz, th, deg2rad(th4));
% Frame {4}
F4 = (F3 * TH4 * Rz4);

% ------------------------------------------------------
% Transformação do frame Inercial para o frame {5}: TH = [Rx][Rz][T]
% ------------------------------------------------------
TH5 = (subs(Rx, th, deg2rad(90)) * subs(Rz, th, deg2rad(-90)) * subs(T, [dx dy dz], [0 0 L4]));
% Transformação para dar liberdade a junta
Rz5 = subs(Rz, th, deg2rad(th5));
% Frame {5}
F5 = (F4 * TH5 * Rz5);

% ------------------------------------------------------
% Transformação do frame Inercial para o frame {6}: TH = [T][Rz][Rx]
% ------------------------------------------------------
TH6 = (subs(T, [dx dy dz], [0 0 L5]) * subs(Rz, th, deg2rad(90)) * subs(Rx, th, deg2rad(-90)));
% Transformação para dar liberdade a junta
Rz6 = subs(Rz, th, deg2rad(th6));
% Frame {6}
F6 = (F5 * TH6 * Rz6);

% ------------------------------------------------------
% Transformação do frame Inercial para o frame {7}: TH = [Rx][Rz][T]
% ------------------------------------------------------
TH7 = (subs(Rx, th, deg2rad(90)) * subs(Rz, th, deg2rad(-90)) * subs(T, [dx dy dz], [0 0 L6]));
% Transformação para dar liberdade a junta
Rz7 = subs(Rz, th, deg2rad(th7));
% Frame {7}
F7 = (F6 * TH7 * Rz7);

% ------------------------------------------------------
% Seção Gráfica
% ------------------------------------------------------
% Vetor para armazenar o trajeto do ultimo link do braço
P_path= [];

for teta = 0:360
    
    % Ângulos (variáveis) das juntas baseadas na variável angular:
    t1 = teta;
    t2 = teta;
    t3 = teta;
    t4 = teta;
    t5 = teta;
    t6 = teta;
    t7 = teta;
    
    % Obtendo valores numéricos para as transformações de frames:
    f1 = double(subs(F1 , th1 , t1));
    f2 = double(subs(F2 , [th1 th2] , [t1 t2]));
    f3 = double(subs(F3 , [th1 th2 th3] , [t1 t2 t3]));
    f4 = double(subs(F4 , [th1 th2 th3 th4] , [t1 t2 t3 t4]));
    f5 = double(subs(F5 , [th1 th2 th3 th4 th5] , [t1 t2 t3 t4 t5]));
    f6 = double(subs(F6 , [th1 th2 th3 th4 th5 th6] , [t1 t2 t3 t4 t5 t6]));
    f7 = double(subs(F7 , [th1 th2 th3 th4 th5 th6 th7] , [t1 t2 t3 t4 t5 t6 t7]));
    
    % Guarda o trajeto descrito por P
    P_path= [P_path, f7(:,4)];

    % MANIPULADOR PALITINHO:
    % Transição do frame {0} para o {1}:
    plot3([F0(1,4) f1(1,4)], [F0(2,4) f1(2,4)], [F0(3,4) f1(3,4)], 'k' , 'linewidth', 4)
    title(['Angulo: ' num2str(teta) '°']); axis equal; grid on; xlabel('x'); ylabel('y'); zlabel('z'); hold on;
    
    % Transição do frame {1} para o {2}:
    plot3([f1(1,4) f2(1,4)], [f1(2,4) f2(2,4)], [f1(3,4) f2(3,4)], 'color',[255,199,9]/256 , 'linewidth', 4)
    
    % Transição do frame {2} para o {3}:
    plot3([f2(1,4) f3(1,4)], [f2(2,4) f3(2,4)], [f2(3,4) f3(3,4)], 'color',[255,199,9]/256 , 'linewidth', 4)
    
    % Transição do frame {3} para o {4}:
    plot3([f3(1,4) f4(1,4)], [f3(2,4) f4(2,4)], [f3(3,4) f4(3,4)], 'color',[255,199,9]/256 , 'linewidth', 4)
    
    % Transição do frame {4} para o {5}:
    plot3([f4(1,4) f5(1,4)], [f4(2,4) f5(2,4)], [f4(3,4) f5(3,4)], 'color',[255,199,9]/256 , 'linewidth', 4)
    
    % Transição do frame {5} para o {6}:
    plot3([f5(1,4) f6(1,4)], [f5(2,4) f6(2,4)], [f5(3,4) f6(3,4)], 'color',[255,199,9]/256 , 'linewidth', 4)
    
    % Transição do frame {6} para o {7}:
    plot3([f6(1,4) f7(1,4)] , [f6(2,4) f7(2,4)], [f6(3,4) f7(3,4)], 'color',[255,199,9]/256 , 'linewidth', 4)
    
    % ------FRAMES ------
    % FRAME {0}:
    % Posição da origem do frame {0}
    plot3(F0(1,4) , F0(2,4) , F0(3,4) , 'sk', 'linewidth', 5 , 'markersize', 10);
    text(F0(1,4) , F0(2,4) , F0(3,4)-0.4 , '\{0\}')
    text(F0(1,4) , F0(2,4) , F0(3,4)+0.25, '- BASE ESTÁTICA')
     
    
    % Eixo-x no robô no frame {0}
    plot3([F0(1,4) F0(1,4)+F0(1,1)] , [F0(2,4) F0(2,4)+F0(2,1)] , [F0(3,4) F0(3,4)+F0(3,1)] , 'b', 'linewidth', 2)
    text((F0(1,4)+F0(1,1)) , (F0(2,4)+F0(2,1)) , (F0(3,4)+F0(3,1)) , 'x_{\{0\}}')
    
    % Eixo-y no robô no frame {0}
    plot3([F0(1,4) F0(1,4)+F0(1,2)] , [F0(2,4) F0(2,4)+F0(2,2)] , [F0(3,4) F0(3,4)+F0(3,2)] , 'r', 'linewidth', 2)
    text((F0(1,4)+F0(1,2)) , (F0(2,4)+F0(2,2)) , (F0(3,4)+F0(3,2)) , 'y_{\{0\}}')
    
    % Eixo-z no robô no frame {0}
    plot3([F0(1,4) F0(1,4)+F0(1,3)] , [F0(2,4) F0(2,4)+F0(2,3)] , [F0(3,4) F0(3,4)+F0(3,3)] , 'g', 'linewidth', 2)
    text((F0(1,4)+F0(1,3)) , (F0(2,4)+F0(2,3)) , (F0(3,4)+F0(3,3)) , 'z_{\{0\}}')
    
    % FRAME {1}:
    % Posição inicial do robô no frame {1}
    plot3(f1(1,4) , f1(2,4) , f1(3,4) , 'or', 'linewidth', 5 , 'markersize', 10);
    text(f1(1,4) , f1(2,4) , f1(3,4)-0.4 , '\{1\}')
    
    % Eixo-x no robô no frame {1}
    plot3([f1(1,4) f1(1,4)+f1(1,1)] , [f1(2,4) f1(2,4)+f1(2,1)] , [f1(3,4) f1(3,4)+f1(3,1)] , 'b', 'linewidth', 2)
    text(f1(1,4)+f1(1,1) , f1(2,4)+f1(2,1) , f1(3,4)+f1(3,1) , 'x_{\{1\}}')
    
    % Eixo-y no robô no frame {1}
    plot3([f1(1,4) f1(1,4)+f1(1,2)] , [f1(2,4) f1(2,4)+f1(2,2)] , [f1(3,4) f1(3,4)+f1(3,2)] , 'r', 'linewidth', 2)
    text(f1(1,4)+f1(1,2) , f1(2,4)+f1(2,2) , f1(3,4)+f1(3,2) , 'y_{\{1\}}')
    
    % Eixo-z no robô no frame {1}
    plot3([f1(1,4) f1(1,4)+f1(1,3)] , [f1(2,4) f1(2,4)+f1(2,3)] , [f1(3,4) f1(3,4)+f1(3,3)] , 'g', 'linewidth', 2)
    text(f1(1,4)+f1(1,3) , f1(2,4)+f1(2,3) , f1(3,4)+f1(3,3) , 'z_{\{1\}}')
    
    % FRAME {2}:
    % Posição inicial do robô no frame {2}
    plot3(f2(1,4) , f2(2,4) , f2(3,4) , 'or', 'linewidth', 5 , 'markersize', 10);
    text(f2(1,4) , f2(2,4) , f2(3,4)-0.4 , '\{2\}')
    
    % Eixo-x no robô no frame {2}
    plot3([f2(1,4) f2(1,4)+f2(1,1)] , [f2(2,4) f2(2,4)+f2(2,1)] , [f2(3,4) f2(3,4)+f2(3,1)] , 'b', 'linewidth', 2)
    text(f2(1,4)+f2(1,1) , f2(2,4)+f2(2,1) , f2(3,4)+f2(3,1) , 'x_{\{2\}}')
    
    % Eixo-y no robô no frame {2}
    plot3([f2(1,4) f2(1,4)+f2(1,2)] , [f2(2,4) f2(2,4)+f2(2,2)] , [f2(3,4) f2(3,4)+f2(3,2)] , 'r', 'linewidth', 2)
    text(f2(1,4)+f2(1,2) , f2(2,4)+f1(2,2) , f2(3,4)+f1(3,2) , 'y_{\{2\}}')
    
    % Eixo-z no robô no frame {2}
    plot3([f2(1,4) f2(1,4)+f2(1,3)] , [f2(2,4) f2(2,4)+f2(2,3)] , [f2(3,4) f2(3,4)+f2(3,3)] , 'g', 'linewidth', 2)
    text(f2(1,4)+f2(1,3) , f2(2,4)+f2(2,3) , f2(3,4)+f2(3,3) , 'z_{\{2\}}')
    
    % FRAME {3}:
    % Posição inicial do robô no frame {3}
    plot3(f3(1,4) , f3(2,4) , f3(3,4) , 'or', 'linewidth', 5 , 'markersize', 10);
    text(f3(1,4) , f3(2,4) , f3(3,4)-0.4 , '\{3\}')
    
    % Eixo-x no robô no frame {3}
    plot3([f3(1,4) f3(1,4)+f3(1,1)] , [f3(2,4) f3(2,4)+f3(2,1)] , [f3(3,4) f3(3,4)+f3(3,1)] , 'b', 'linewidth', 2)
    text(f3(1,4)+f3(1,1) , f3(2,4)+f3(2,1) , f3(3,4)+f3(3,1) , 'x_{\{3\}}')
    
    % Eixo-y no robô no frame {3}
    plot3([f3(1,4) f3(1,4)+f3(1,2)] , [f3(2,4) f3(2,4)+f3(2,2)] , [f3(3,4) f3(3,4)+f3(3,2)] , 'r', 'linewidth', 2)
    text(f3(1,4)+f3(1,2) , f3(2,4)+f3(2,2) , f3(3,4)+f3(3,2) , 'y_{\{3\}}')
    
    % Eixo-z no robô no frame {3}
    plot3([f3(1,4) f3(1,4)+f3(1,3)] , [f3(2,4) f3(2,4)+f3(2,3)] , [f3(3,4) f3(3,4)+f3(3,3)] , 'g', 'linewidth', 2)
    text(f3(1,4)+f3(1,3) , f3(2,4)+f3(2,3) , f3(3,4)+f3(3,3)+0.2 , 'z_{\{3\}}')
    
    % FRAME {4}:
    % Posição inicial do robô no frame {4}
    plot3(f4(1,4) , f4(2,4) , f4(3,4) , 'or', 'linewidth', 5 , 'markersize', 10);
    text(f4(1,4) , f4(2,4) , f4(3,4)-0.4 , '\{4\}')
    
    % Eixo-x no robô no frame {4}
    plot3([f4(1,4) f4(1,4)+f4(1,1)] , [f4(2,4) f4(2,4)+f4(2,1)] , [f4(3,4) f4(3,4)+f4(3,1)] , 'b', 'linewidth', 2)
    text(f4(1,4)+f4(1,1) , f4(2,4)+f4(2,1) , f4(3,4)+f4(3,1) , 'x_{\{4\}}')
    
    % Eixo-y no robô no frame {4}
    plot3([f4(1,4) f4(1,4)+f4(1,2)] , [f4(2,4) f4(2,4)+f4(2,2)] , [f4(3,4) f4(3,4)+f4(3,2)] , 'r', 'linewidth', 2)
    text(f4(1,4)+f4(1,2) , f4(2,4)+f4(2,2) , f4(3,4)+f4(3,2) , 'y_{\{4\}}')
    
    % Eixo-z no robô no frame {4}
    plot3([f4(1,4) f4(1,4)+f4(1,3)] , [f4(2,4) f4(2,4)+f4(2,3)] , [f4(3,4) f4(3,4)+f4(3,3)] , 'g', 'linewidth', 2)
    text(f4(1,4)+f4(1,3) , f4(2,4)+f4(2,3) , f4(3,4)+f4(3,3) , 'z_{\{4\}}')
    
    % FRAME {5}:
    % Posição inicial do robô no frame {4}
    plot3(f5(1,4) , f5(2,4) , f5(3,4) , 'or', 'linewidth', 5 , 'markersize', 10);
    text(f5(1,4) , f5(2,4) , f5(3,4)-0.4 , '\{5\}')
    
    % Eixo-x no robô no frame {4}
    plot3([f5(1,4) f5(1,4)+f5(1,1)] , [f5(2,4) f5(2,4)+f5(2,1)] , [f5(3,4) f5(3,4)+f5(3,1)] , 'b', 'linewidth', 2)
    text(f5(1,4)+f5(1,1) , f5(2,4)+f5(2,1) , f5(3,4)+f5(3,1) , 'x_{\{5\}}')
    
    % Eixo-y no robô no frame {4}
    plot3([f5(1,4) f5(1,4)+f5(1,2)] , [f5(2,4) f5(2,4)+f5(2,2)] , [f5(3,4) f5(3,4)+f5(3,2)] , 'r', 'linewidth', 2)
    text(f5(1,4)+f5(1,2) , f5(2,4)+f5(2,2) , f5(3,4)+f5(3,2) , 'y_{\{5\}}')
    
    % Eixo-z no robô no frame {4}
    plot3([f5(1,4) f5(1,4)+f5(1,3)] , [f5(2,4) f5(2,4)+f5(2,3)] , [f5(3,4) f5(3,4)+f5(3,3)] , 'g', 'linewidth', 2)
    text(f5(1,4)+f5(1,3) , f5(2,4)+f5(2,3) , f5(3,4)+f5(3,3) , 'z_{\{5\}}')
    
    % FRAME {6}:
    % Posição inicial do robô no frame {4}
    plot3(f6(1,4) , f6(2,4) , f6(3,4) , 'or', 'linewidth', 5 , 'markersize', 10);
    text(f6(1,4) , f6(2,4) , f6(3,4)-0.4 , '\{6\}')
    
    % Eixo-x no robô no frame {4}
    plot3([f6(1,4) f6(1,4)+f6(1,1)] , [f6(2,4) f6(2,4)+f6(2,1)] , [f6(3,4) f6(3,4)+f6(3,1)] , 'b', 'linewidth', 2)
    text(f6(1,4)+f6(1,1) , f6(2,4)+f6(2,1) , f6(3,4)+f6(3,1) , 'x_{\{6\}}')
    
    % Eixo-y no robô no frame {4}
    plot3([f6(1,4) f6(1,4)+f6(1,2)] , [f6(2,4) f6(2,4)+f6(2,2)] , [f6(3,4) f6(3,4)+f6(3,2)] , 'r', 'linewidth', 2)
    text(f6(1,4)+f6(1,2) , f6(2,4)+f6(2,2) , f6(3,4)+f6(3,2) , 'y_{\{6\}}')
    
    % Eixo-z no robô no frame {4}
    plot3([f6(1,4) f6(1,4)+f6(1,3)] , [f6(2,4) f6(2,4)+f6(2,3)] , [f6(3,4) f6(3,4)+f6(3,3)] , 'g', 'linewidth', 2)
    text(f6(1,4)+f6(1,3) , f6(2,4)+f6(2,3) , f6(3,4)+f6(3,3) , 'z_{\{6\}}')
    
    % FRAME {7}:
    % Posição inicial do robô no frame {4}
    plot3(f7(1,4) , f7(2,4) , f7(3,4) , 'or', 'linewidth', 5 , 'markersize', 10);
    text(f7(1,4) , f7(2,4) , f7(3,4)-0.4 , '\{7\}')
    
    % Eixo-x no robô no frame {4}
    plot3([f7(1,4) f7(1,4)+f7(1,1)] , [f7(2,4) f7(2,4)+f7(2,1)] , [f7(3,4) f7(3,4)+f7(3,1)] , 'b', 'linewidth', 2)
    text(f7(1,4)+f7(1,1) , f7(2,4)+f7(2,1) , f7(3,4)+f7(3,1) , 'x_{\{7\}}')
    
    % Eixo-y no robô no frame {4}
    plot3([f7(1,4) f7(1,4)+f7(1,2)] , [f7(2,4) f7(2,4)+f7(2,2)] , [f7(3,4) f7(3,4)+f7(3,2)] , 'r', 'linewidth', 2)
    text(f7(1,4)+f7(1,2) , f7(2,4)+f7(2,2) , f7(3,4)+f7(3,2) , 'y_{\{7\}}')
    
    % Eixo-z no robô no frame {4}
    plot3([f7(1,4) f7(1,4)+f7(1,3)] , [f7(2,4) f7(2,4)+f7(2,3)] , [f7(3,4) f7(3,4)+f7(3,3)] , 'g', 'linewidth', 2)
    text(f7(1,4)+f7(1,3) , f7(2,4)+f7(2,3) , f7(3,4)+f7(3,3) , 'z_{\{7\}}')
    
    plot3(P_path(1,:) , P_path(2,:) , P_path(3,:) , 'b')
    
    hold off;
    drawnow;
%     gif(grafico, nome_gif);    
end

