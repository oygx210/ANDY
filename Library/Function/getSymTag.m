function output=getSymTag(inSym,inChar)
    output=[];

    symChar=char(inSym);
    tagChar=strcat('__',inChar,'_');
    tagLength=length(tagChar);
    
    tagStart=strfind(symChar,tagChar);
    if(numel(tagStart)~=1)
        return;
    end
    tagEnd=tagStart+tagLength;
    
    tagContent=symChar(tagEnd:end);
    tagContentEnd=strfind(tagContent,'_');
    tagContentEnd=tagContentEnd(1)+numel(symChar)-numel(tagContent);
    
    output=symChar(tagEnd:tagContentEnd-1);
end