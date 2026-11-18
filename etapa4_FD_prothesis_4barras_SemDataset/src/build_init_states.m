function P = build_init_states()
%BUILD_INIT_STATES Banco de estados iniciais da etapa 4, tirado de marchas
%   SIMULADAS na etapa 3 (avaliacoes com beta <= 0,1 que andaram 10 s).
%   Nao usa o dataset: sao estados que o proprio modelo atingiu andando.
%   Saida: data/init_states.mat com qraw, dep, velocidades (dif. finitas),
%   excitacoes (usadas como ativacao inicial) e a origem de cada amostra.
here = fileparts(mfilename('fullpath'));
src = fullfile(here, '..', '..', 'etapa3_FD_prothesis_4barras', 'results');
f = dir(fullfile(src, 'chunk*_traj.mat'));
f = f(~cellfun(@isempty, regexp({f.name}, '^chunk\d+_b0(00|05|10)_traj\.mat$')));
P = struct('q', [], 'qd', [], 'dep', [], 'depd', [], 'e', [], 'ty', [], 'src', strings(0,1));
for i = 1:numel(f)
    S = load(fullfile(f(i).folder, f(i).name), 'tr'); tr = S.tr;
    if tr.t(end) < 9.9, continue, end
    dt = 0.01;
    qd = gradient(tr.qraw', dt)'; dd = gradient(tr.dep', dt)';
    k = find(tr.t >= 1.0 & tr.t <= tr.t(end) - 0.5); k = k(1:5:end);
    P.q = [P.q; tr.qraw(k,:)]; P.qd = [P.qd; qd(k,:)];
    P.dep = [P.dep; tr.dep(k,:)]; P.depd = [P.depd; dd(k,:)];
    P.e = [P.e; tr.e(k,:)]; P.ty = [P.ty; tr.qraw(k,5)];
    P.src = [P.src; repmat(string(f(i).name), numel(k), 1)];
    fprintf('%s: %d estados\n', f(i).name, numel(k));
end
fprintf('total: %d estados | pelvis_ty %.3f..%.3f (mediana %.3f) | vx mediana %.2f m/s\n', ...
    size(P.q,1), min(P.ty), max(P.ty), median(P.ty), median(P.qd(:,4)));
save(fullfile(here, '..', 'data', 'init_states.mat'), '-struct', 'P');
end
