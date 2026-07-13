function generate_word_manual(file_name)
%GENERATE_WORD_MANUAL Create the DOCX user manual without extra toolboxes.
% The manual text is stored as UTF-8 in manual_content_utf8.txt so MATLAB
% source encoding cannot corrupt Chinese characters.

content_file = fullfile(fileparts(mfilename('fullpath')), 'manual_content_utf8.txt');
if ~isfile(content_file)
    error('Manual content file not found: %s', content_file);
end

raw = read_utf8_text(content_file);
lines = regexp(raw, '\r\n|\n|\r', 'split');

target_dir = fileparts(file_name);
if ~isempty(target_dir) && ~isfolder(target_dir)
    mkdir(target_dir);
end

tmpRoot = tempname;
mkdir(tmpRoot);
mkdir(fullfile(tmpRoot, '_rels'));
mkdir(fullfile(tmpRoot, 'word'));
mkdir(fullfile(tmpRoot, 'word', '_rels'));

write_text(fullfile(tmpRoot, '[Content_Types].xml'), content_types_xml());
write_text(fullfile(tmpRoot, '_rels', '.rels'), rels_xml());
write_text(fullfile(tmpRoot, 'word', '_rels', 'document.xml.rels'), document_rels_xml());
write_text(fullfile(tmpRoot, 'word', 'styles.xml'), styles_xml());
write_text(fullfile(tmpRoot, 'word', 'numbering.xml'), numbering_xml());
write_text(fullfile(tmpRoot, 'word', 'document.xml'), document_xml(lines));

cwd = pwd;
cleanupObj = onCleanup(@() cleanup_temp(cwd, tmpRoot));
cd(tmpRoot);
zip_file = fullfile(tmpRoot, 'manual_docx_package.zip');
zip(zip_file, {'[Content_Types].xml', '_rels', 'word'});
cd(cwd);
if isfile(file_name)
    delete(file_name);
end
movefile(zip_file, file_name);
end

function txt = read_utf8_text(file_name)
fid = fopen(file_name, 'r', 'n', 'UTF-8');
if fid < 0
    error('Cannot read %s', file_name);
end
cleanupObj = onCleanup(@() fclose(fid));
bytes = fread(fid, '*uint8')';
txt = native2unicode(bytes, 'UTF-8');
end

function cleanup_temp(cwd, tmpRoot)
if isfolder(cwd)
    cd(cwd);
end
if isfolder(tmpRoot)
    rmdir(tmpRoot, 's');
end
end

function write_text(file_name, txt)
fid = fopen(file_name, 'wt', 'n', 'UTF-8');
if fid < 0
    error('Cannot write %s', file_name);
end
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, '%s', txt);
end

function txt = content_types_xml()
txt = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>' ...
    '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">' ...
    '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>' ...
    '<Default Extension="xml" ContentType="application/xml"/>' ...
    '<Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>' ...
    '<Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>' ...
    '<Override PartName="/word/numbering.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.numbering+xml"/>' ...
    '</Types>'];
end

function txt = rels_xml()
txt = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>' ...
    '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">' ...
    '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>' ...
    '</Relationships>'];
end

function txt = document_rels_xml()
txt = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>' ...
    '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">' ...
    '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/numbering" Target="numbering.xml"/>' ...
    '</Relationships>'];
end

function txt = numbering_xml()
txt = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>' ...
    '<w:numbering xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">' ...
    '<w:abstractNum w:abstractNumId="1"><w:multiLevelType w:val="singleLevel"/>' ...
    '<w:lvl w:ilvl="0"><w:start w:val="1"/><w:numFmt w:val="bullet"/><w:lvlText w:val="•"/>' ...
    '<w:lvlJc w:val="left"/><w:pPr><w:tabs><w:tab w:val="num" w:pos="720"/></w:tabs><w:ind w:left="720" w:hanging="360"/></w:pPr>' ...
    '<w:rPr><w:rFonts w:ascii="Symbol" w:hAnsi="Symbol" w:hint="default"/></w:rPr></w:lvl></w:abstractNum>' ...
    '<w:num w:numId="1"><w:abstractNumId w:val="1"/></w:num>' ...
    '</w:numbering>'];
end

function txt = styles_xml()
txt = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>' ...
    '<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">' ...
    style_xml('Normal', 'Normal', 'Microsoft YaHei', 'Times New Roman', '21', false, '000000', '0', '120', '300') ...
    style_xml('Title', 'Title', 'Microsoft YaHei', 'Times New Roman', '36', true, '0B2545', '0', '260', '320') ...
    style_xml('Heading1', 'heading 1', 'Microsoft YaHei', 'Times New Roman', '28', true, '1F4D78', '240', '120', '300') ...
    style_xml('Heading2', 'heading 2', 'Microsoft YaHei', 'Times New Roman', '24', true, '2E74B5', '180', '80', '300') ...
    style_xml('Bullet', 'Bullet', 'Microsoft YaHei', 'Times New Roman', '21', false, '000000', '0', '80', '300') ...
    '</w:styles>'];
end

function txt = style_xml(styleId, name, eastAsia, asciiFont, sz, bold, color, before, after, line)
bold_xml = '';
if bold
    bold_xml = '<w:b/>';
end
txt = ['<w:style w:type="paragraph" w:styleId="' styleId '">' ...
    '<w:name w:val="' name '"/>' ...
    '<w:pPr><w:spacing w:before="' before '" w:after="' after '" w:line="' line '" w:lineRule="auto"/></w:pPr>' ...
    '<w:rPr><w:rFonts w:eastAsia="' eastAsia '" w:ascii="' asciiFont '" w:hAnsi="' asciiFont '"/>' ...
    bold_xml '<w:color w:val="' color '"/><w:sz w:val="' sz '"/></w:rPr>' ...
    '</w:style>'];
end

function txt = document_xml(lines)
body = '';
for i = 1:numel(lines)
    p = strtrim(lines{i});
    if isempty(p)
        body = [body paragraph_xml('', 'Normal', false)]; %#ok<AGROW>
        continue;
    end
    if starts_with(p, '# ')
        body = [body paragraph_xml(p(3:end), 'Title', false)]; %#ok<AGROW>
    elseif starts_with(p, '## ')
        body = [body paragraph_xml(p(4:end), 'Heading1', false)]; %#ok<AGROW>
    elseif starts_with(p, '### ')
        body = [body paragraph_xml(p(5:end), 'Heading2', false)]; %#ok<AGROW>
    elseif starts_with(p, '- ')
        body = [body paragraph_xml(p(3:end), 'Bullet', true)]; %#ok<AGROW>
    else
        body = [body paragraph_xml(p, 'Normal', false)]; %#ok<AGROW>
    end
end
txt = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>' ...
    '<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">' ...
    '<w:body>' body '<w:sectPr><w:pgSz w:w="11906" w:h="16838"/><w:pgMar w:top="1440" w:right="1440" w:bottom="1440" w:left="1440"/></w:sectPr></w:body></w:document>'];
end

function txt = paragraph_xml(text, style, bullet)
indent = '';
if bullet
    indent = '<w:numPr><w:ilvl w:val="0"/><w:numId w:val="1"/></w:numPr>';
end
txt = ['<w:p><w:pPr><w:pStyle w:val="' style '"/>' indent '</w:pPr>' ...
    '<w:r><w:rPr><w:rFonts w:eastAsia="Microsoft YaHei" w:ascii="Times New Roman" w:hAnsi="Times New Roman"/></w:rPr>' ...
    '<w:t xml:space="preserve">' xml_escape(text) '</w:t></w:r></w:p>'];
end

function tf = starts_with(s, pat)
tf = numel(s) >= numel(pat) && strcmp(s(1:numel(pat)), pat);
end

function s = xml_escape(s)
s = strrep(s, '&', '&amp;');
s = strrep(s, '<', '&lt;');
s = strrep(s, '>', '&gt;');
s = strrep(s, '"', '&quot;');
s = strrep(s, '''', '&apos;');
end
