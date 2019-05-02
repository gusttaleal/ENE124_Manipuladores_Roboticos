%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
% Função para executar as tranformações de translação e rotação
%       | EIXO DE ||
%       | ROTAÇÃO ||TRANSLAÇÃO|
%       | X  Y  Z ||D
%
%     1 | Px Px Px Dx |
% M = 2 | Py Py Py Dy |
%     3 | Pz Pz Pz Dz |
%     4 | 0  0  0  1  |
%         1  2  3  4 

function [Fy] = transformation(Fx, Lz, Tz, Lx, Tx)
syms th dx dy dz 

% Matriz de translação (generalista)
T = [1 0 0 dx; 0 1 0 dy; 0 0 1 dz; 0 0 0 1];
% Matriz de rotação em x (generalista)
Rx = [1 0 0 0; 0 cos(th) -sin(th) 0; 0 sin(th) cos(th) 0; 0 0 0 1];
% Matriz de rotação em z (generalista)
Rz = [cos(th) -sin(th) 0 0; sin(th) cos(th) 0 0; 0 0 1 0; 0 0 0 1];

% ----------------------------------------------------------------
% Transformação do frame {Fx} para o frame {Fy}: TH = [Tz][Rz][Tx][Rx]
% ----------------------------------------------------------------
% Transformação em Z
THz = subs(T, [dx dy dz], [0 0 Lz]) * subs(Rz, th, deg2rad(Tz));
% Transformação em X
THx = subs(T, [dx dy dz], [Lx 0 0]) * subs(Rx, th, deg2rad(Tx));
% Transformação Homogênea
TH1 = THz * THx;

% Frame {Y}
Fy = (Fx * TH1);
end