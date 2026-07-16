%SEASTACK_EXTERNAL  SEA-Stack external force module client (MATLAB).
%
% Speaks the v1 length-prefixed JSON protocol over TCP to 127.0.0.1.
% See docs/extending/EXTERNAL_FORCE_MODULES.md.
%
% Usage:
%   seastack_external(@my_evaluate, @my_initialize)   % function handles
%   seastack_external('linear_damper')                % built-in example
%
% SEA-Stack launches MATLAB with --seastack-port <N> appended; this script
% parses that from the command line when run via:
%   matlab -batch "seastack_external('linear_damper')"
% and expects SEASTACK_PORT in the environment, or argv via getenv.

function seastack_external(model_id, init_fn)
    if nargin < 1 || isempty(model_id)
        model_id = 'linear_damper';
    end

    port = resolve_port();
    if port <= 0
        error('seastack_external: missing --seastack-port / SEASTACK_PORT');
    end

    state = struct('damping', 50.0, 'name', 'MatlabLinearDamper', 'version', '1.0');

    tcp = tcpclient('127.0.0.1', port, 'Timeout', 30);
    cleanup = onCleanup(@() delete(tcp)); %#ok<NASGU>

    running = true;
    while running
        msg = recv_message(tcp);
        op = msg.op;
        switch op
            case 'initialize'
                if isfield(msg, 'config') && isstruct(msg.config) && isfield(msg.config, 'damping')
                    state.damping = double(msg.config.damping);
                end
                if nargin >= 2 && ~isempty(init_fn)
                    state = init_fn(msg, state);
                end
                reply = struct('status', 'ok', 'protocol', 1, ...
                    'name', state.name, 'version', state.version, 'n_states', 0);
                send_message(tcp, reply);
            case 'evaluate'
                inputs = double(msg.in(:))';
                if ischar(model_id) || isstring(model_id)
                    out = local_evaluate(char(model_id), msg.t, msg.dt, inputs, state);
                else
                    out = model_id(msg.t, msg.dt, inputs, state);
                end
                reply = struct('status', 'ok', 'out', out);
                send_message(tcp, reply);
            case {'reset', 'commit', 'rollback'}
                send_message(tcp, struct('status', 'ok'));
            case 'shutdown'
                send_message(tcp, struct('status', 'ok'));
                running = false;
            otherwise
                send_message(tcp, struct('status', 'error', ...
                    'message', sprintf('unknown op: %s', op)));
        end
    end
end

function port = resolve_port()
    port = 0;
    env = getenv('SEASTACK_PORT');
    if ~isempty(env)
        port = str2double(env);
        return;
    end
    % Parse matlab -batch argv via undocumented feature when available.
    try
        args = feature('getArguments'); %#ok<*NASGU>
    catch
        args = {};
    end
    if iscell(args)
        for i = 1:numel(args)
            if strcmp(args{i}, '--seastack-port') && i < numel(args)
                port = str2double(args{i+1});
                return;
            end
        end
    end
end

function out = local_evaluate(model_id, t, dt, inputs, state) %#ok<INUSL>
    switch model_id
        case 'linear_damper'
            vel = 0.0;
            if numel(inputs) >= 2
                vel = inputs(2);
            end
            out = -state.damping * vel;
        otherwise
            error('Unknown model_id: %s', model_id);
    end
end

function msg = recv_message(tcp)
    hdr = read(tcp, 4, 'uint8');
    len = typecast(fliplr(uint8(hdr(:)')), 'uint32');  % big-endian
    if len > 0
        payload = read(tcp, double(len), 'uint8');
        msg = jsondecode(native2unicode(payload, 'UTF-8'));
    else
        msg = struct();
    end
end

function send_message(tcp, obj)
    payload = unicode2native(jsonencode(obj), 'UTF-8');
    len = uint32(numel(payload));
    hdr = fliplr(typecast(len, 'uint8'));  % big-endian
    write(tcp, hdr);
    write(tcp, payload);
end
