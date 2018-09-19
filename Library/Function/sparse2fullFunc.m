function outContent=sparse2fullFunc(inContent)
    replaceContent={
                    'out1=zeros(outSize);';
                    'for eleNum=1:numel(elementList)';
                    '    out1(elementRow(eleNum),elementCol(eleNum))=elementList(eleNum);';
                    'end';
                    };

    outputStatement=inContent{end};
    begin=strfind(outputStatement,'([');
    midpoint=strfind(outputStatement,'],');
    final=strfind(outputStatement,');');
    rowdata=outputStatement(begin+1:midpoint(1));
    coldata=outputStatement(midpoint(1)+2:midpoint(2));
    elementdata=outputStatement(midpoint(2)+2:midpoint(3));
    sizedata=strcat('[',outputStatement(midpoint(3)+2:final-1),']');
    outContent=[...
                inContent(1:end-1);
                {
                    strcat('outSize=',sizedata,';');
                    strcat('elementRow=',rowdata,';');
                    strcat('elementCol=',coldata,';');
                    strcat('elementList=',elementdata,';');
                    };...
                replaceContent];
    outContent=strrep(outContent,'sparse','');
end