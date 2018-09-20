A = [0, 0, 1, 0;
     0, 0, 0, 1;
     17.7828531704201,  0, -0.00858417221300891,  0.00858417221300891;
     47.8688365622367,  0, 0.0288387521185202,  -0.0288387521185202];

B = [0;
     0;
     -0.0858417221300890;
     0.288387521185202];
 
Q = 300*[1 0 0 0; 0 320 0 0; 0 0 100 0; 0 0 0 300];
R = 500*eye(1);

%% my failed lqr attempt
Z = [A, -B*(R\B'); -Q, -A'];
[U,T] = schur(Z,'real');
X = U(5:end, 1:4)/U(1:4,1:4);
K = R\(B'*X);

%% Built-in LQR
[F, S, e] = lqr(A, B, Q, R);

%% replicating built-in lqr
%[S,E,KK,report] = care(A,B,Q,R,zeros(size(B)),[]);
% begin care
    [n,m] = size(B); 
    S = zeros(size(B));
    E = eye(n);
    D = diag(R);
    U = 1;
    DINV = diag(1./D);
    BU = B * U;
    SU = S * U;
    AS = A - BU * DINV * SU';
    H12 = -BU * DINV * BU';
    H21 = -Q + SU * DINV * SU';

    H = [AS (H12+H12')/2; (H21+H21')/2 -AS'];
    J = [];

    %[X,L,Report] = gcare(H,J,n);
    % begin gcare
        n2=2*n;
        %[H,J,D,perm] = arescale(H,J,n);
        % begin arescale
            m = size(H,1)-n2;
            TolZero = 1e2*eps;
            M = H;
            mu = abs(diag(M));
            mu = mu(:,ones(n2,1));
            M(abs(M)<TolZero*(mu+mu')) = 0;
            %[s,perm] = mscale(M,'perm','fullbal');
            % begin mscale
                dM = diag(diag(M));
                [s,~,b] = balance(M-dM,'noperm');
                b = b+dM;
                nM = size(M,1);
                perm = (1:nM)';
                ip = triperm('H',M);
                b = b(ip,ip);
                perm(ip) = perm;
            % end mscale
            perm = perm(perm<=n2);
            perm(perm) = 1:n2;
            s = log2(s); 
            D = round((-s(1:n)+s(n+1:n2))/2);
            s = pow2([D ; -D ; -s(n2+1:n2+m)]);
            D = s(1:n); 
            H = lrscale(H,s,1./s);
        % end arescale
        [z,t] = schur(H(perm,perm),'real');
        [z(perm,:),t,Success] = ordschur(z,t,'lhp');
        L = ordeig(t);
        X1 = z(1:n,1:n);
        X2 = z(n+1:n2,1:n);
        Report = [];%arecheckout(X1,X2,Success,(real(L)<0));   
        % [X,Report] = arefact2x(X1,X2,D,Report);
        % begin arefact2x
            [l,u,p] = lu(X1,'vector');
            X(:,p) = (X2/u)/l;
            X = (X+X')/2;
            X = lrscale(X,D,D);
        % end arefact2x
    % end gcare
    G = R\(B'*X*E+S');
% end care

%% simplifying built-in lqr
%[S,E,KK,report] = care(A,B,Q,R,zeros(size(B)),[]);
% begin care
    H = [A -B*(R\B'); -Q -A'];
    eig(H)
    J = [];

    %[X,L,Report] = gcare(H,J,n);
    % begin gcare
        n2=2*n;
        %[H,J,D,perm] = arescale(H,J,n);
        % begin arescale
            M = H;
            %[s,perm] = mscale(M,'perm','fullbal');
            % begin mscale
                dM = diag(diag(M));
                [s,~,b] = balance(M-dM,'noperm');
            % end mscale
            s = log2(s); 
            D = round((-s(1:n)+s(n+1:n2))/2);
            s = pow2([D ; -D]);
            D = s(1:n); 
            H = lrscale(H,s,1./s);
        % end arescale
        disp(perm);
        [z,t] = schur(H,'real');
        [z(perm,:),t,Success] = ordschur(z,t,'lhp');
        L = ordeig(t);
        X1 = z(1:n,1:n);
        X2 = z(n+1:n2,1:n);
        Report = [];%arecheckout(X1,X2,Success,(real(L)<0));   
        % [X,Report] = arefact2x(X1,X2,D,Report);
        % begin arefact2x
            [l,u,p] = lu(X1,'vector');
            X(:,p) = (X2/u)/l;
            X = (X+X')/2;
            X = lrscale(X,D,D);
        % end arefact2x
    % end gcare
    G2 = R\(B'*X);
% end care


%% compare
disp([K', F', G', G2']);
