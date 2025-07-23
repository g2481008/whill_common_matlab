classdef SharedMem < handle
    %==================================================================
    %   SharedMem  ─ ultra-light shared memory helper
    %   • Minimal modification to support symmetric startup.
    %==================================================================
    %% ────────────────────────── CONSTANTS ──────────────────────────
    properties (Constant, Access = private)
        typeBytes = struct( ...
            'double',8,'single',4, ...
            'uint64',8,'int64',8,'uint32',4,'int32',4, ...
            'uint16',2,'int16',2,'uint8',1,'int8',1);
        
    end
    %% ─────────────────────────── FIELDS ────────────────────────────
    properties (Access = public)
        mm                  % memmapfile object
        varInfo             % struct array per variable
        oneBufBytes double  % bytes per buffer (double for safe math)
        hdrLen      double  % header length
        buf0Start   double  % start index of buffer-0 in the payload
        timeout = 0.05;
    end
    properties (Access=private)
        seq = uint32(0);

    end
    %% ────────────────────────── PUBLIC API ─────────────────────────
    methods
        %======================= CONSTRUCTOR =========================
        function obj = SharedMem(fname,checkBin,vars)
            persistent uuid
            for fid = openedFiles
                if strcmp(fname, fopen(fid)), fclose(fid); end
            end
            if nargin == 3 && isfile(fname)
                delete(fname);
            end
            if nargin == 3                    % === Writer ===
                header  = bridge.SharedMem.makeHeader(vars);
                obj.hdrLen = double(numel(header));
                [obj.varInfo,obj.oneBufBytes] = bridge.SharedMem.buildInfo(vars);
                obj.buf0Start = 47 + obj.hdrLen;        % [busy buf readerReady(1B) seq(4B) uuid(36B) hdrLen(4B) hdr]
                totalBytes = obj.buf0Start + 2*obj.oneBufBytes;
                uuid = char(java.util.UUID.randomUUID().toString());
                bridge.SharedMem.createFile(fname, header, totalBytes,uuid);
            else                               % === Reader ===
                uuid = bridge.SharedMem.waitFile(fname,uuid,checkBin);
                [header,obj.hdrLen] = bridge.SharedMem.readHeader(fname);
                vars = bridge.SharedMem.json2vars(header);
                [obj.varInfo,obj.oneBufBytes] = bridge.SharedMem.buildInfo(vars);
                obj.buf0Start = 47 + obj.hdrLen;
                totalBytes = obj.buf0Start + 2*obj.oneBufBytes;
            end
            fmt = {'uint8',[1 totalBytes],'data'};
            writableFlag = true;
            obj.mm = memmapfile(fname,'Writable',writableFlag,'Format',fmt);

            if nargin ~= 3 % Reader is Ready!
                obj.mm.Data(1).data(3) = 1; 
            else % Waiting Reader is Ready.
                while obj.mm.Data(1).data(3) == 0, pause(0.01); end
            end
        end
        %========================== WRITE ============================
        function seq32 = write(obj,data,varargin)
            while obj.mm.Data(1).data(1) ==1, end
            obj.mm.Data(1).data(1) = 1;
            tgt = 1 - obj.mm.Data(1).data(2);
            base = double(obj.buf0Start) + double(tgt)*obj.oneBufBytes;
            for k = 1:numel(obj.varInfo)
                vi  = obj.varInfo(k);
                val = data.(vi.name);
                if any(size(val) > vi.maxSz)
                    error("SharedMem:SizeExceeded", "%s exceeds maxSize %s", vi.name, mat2str(vi.maxSz));
                end
                idxMeta = double(base) + double(vi.metaOff) + (1:double(vi.metaBytes));
                obj.mm.Data(1).data(idxMeta) = typecast(uint32(size(val)),'uint8');
                raw = typecast(cast(val(:),vi.type),'uint8');
                idxDat = double(base) + double(vi.dataOff) + (1:double(numel(raw)));
                obj.mm.Data(1).data(idxDat) = raw;
            end
            obj.seq = obj.seq + 1;            
            seq32 = uint32(obj.seq);
            if nargin > 2 && obj.seq <= varargin{1}
                seq32 = uint32(varargin{1}); 
            end
            obj.mm.Data(1).data(4:7) = typecast(seq32,'uint8');            
            obj.mm.Data(1).data(2) = tgt;
            obj.mm.Data(1).data(1) = 0;
        end
        %========================== READ =============================
        function [out,ok] = read(obj,skipSame)
            persistent lastBuf
            if nargin < 2, skipSame = false; end
            out = []; ok = false;
            tStart = tic;
            while toc(tStart) < obj.timeout
                if obj.mm.Data(1).data(1)==1, continue; end
                buf = obj.mm.Data(1).data(2);
                if skipSame && isequal(buf,lastBuf), continue; end
                nowseq = typecast(obj.mm.Data(1).data(4:7),'uint32');
                if nowseq <= obj.seq, continue; end
                
                base = double(obj.buf0Start) + double(buf)*obj.oneBufBytes;
                for k = 1:numel(obj.varInfo)
                    vi = obj.varInfo(k);
                    idxMeta = double(base) + double(vi.metaOff) + (1:double(vi.metaBytes));
                    sz      = typecast(obj.mm.Data(1).data(idxMeta),'uint32');
                    if any(sz==0), continue; end
                    ne  = prod(double(sz));
                    idxRaw = base + vi.dataOff + (1:ne*vi.bpe);
                    raw = obj.mm.Data(1).data(idxRaw);
                    out.(vi.name) = reshape(typecast(raw,vi.type),sz);
                end
                obj.seq = nowseq;
                out.sequence = obj.seq;
                ok = true;
                lastBuf = buf;
                return;
            end
        end
    end
    %% ────────────────────── INTERNAL HELPERS ──────────────────────
    methods (Static, Access = private)
        %------------- make/parse/build ------------------
        function bytes = makeHeader(vars)
            s = repmat(struct('name',[],'type',[],'maxSize',[]), size(vars,1), 1);
            for i = 1:size(vars,1), s(i) = struct('name',vars{i,1},'type',vars{i,2},'maxSize',vars{i,3}); end
            bytes = uint8(jsonencode(s));
        end
        function vars = json2vars(hdrBytes)
            s = jsondecode(char(hdrBytes(:).')); 
            vars = cell(numel(s),3);
            for i = 1:numel(s), vars{i,1}=s(i).name; vars{i,2}=s(i).type; vars{i,3}=s(i).maxSize; end
        end
        function [info,one] = buildInfo(vars)
            offset = 0;
            info = repmat(struct('name',[],'type',[],'bpe',[],'maxSz',[],'metaOff',[],'metaBytes',[],'dataOff',[],'totalBytes',[]), size(vars,1), 1);
            for i = 1:size(vars,1)
                tp=vars{i,2}; sz=vars{i,3}; bpe=double(bridge.SharedMem.typeBytes.(tp)); mb=double(numel(sz)*4); db=double(prod(sz)*bpe);
                info(i) = struct('name',vars{i,1},'type',tp,'bpe',bpe,'maxSz',sz,'metaOff',offset,'metaBytes',mb,'dataOff',offset+mb,'totalBytes',mb+db);
                offset = offset + mb + db;
            end
            one = double(offset);
        end
        %------------- create & zero-fill file ----------------------
        function createFile(fname,hdr,totalBytes,uuid)
            hdrLen = uint32(numel(hdr));
            f = fopen(fname,'w+b');
            fwrite(f, zeros(1,totalBytes,'uint8'),'uint8');
            fseek(f,0,'bof');
            fwrite(f,uint8([0 0 0]),'uint8');  % busy, buf, readerReady
            fwrite(f,uint32(0),'uint32'); % seq
            fwrite(f,uint8(uuid),'uint8'); % uuid
            fwrite(f,hdrLen,'uint32');
            fwrite(f,hdr,'uint8');
            fclose(f);
        end
        %------------- read header ---------------------------------
        function [hdr,hLen] = readHeader(fname)
            f = fopen(fname,'r');
            fseek(f,43,'bof');
            hLen = fread(f,1,'uint32');
            hdr  = fread(f,hLen,'uint8');
            fclose(f);
        end
        %------------- wait until writer created file --------------
        function uuid = waitFile(fname,uuid,checkBin)
            while ~isfile(fname), pause(0.01); end
            fuuid = [];
            while true
                f = fopen(fname,'r');
                b = fread(f,43,'uint8=>uint8');
                if checkBin, fuuid = char(b(8:43)); end
                ok = ~isempty(b) && b(1)==0 && (isempty(uuid) || ~strcmp(fuuid,uuid));
                hLen = fread(f,1,'uint32');
                fclose(f);
                if ok && hLen>0, uuid=fuuid; break; end
                pause(0.01);
            end
        end
    end
end
