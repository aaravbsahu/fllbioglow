// Google Apps Script to receive motor positions and deltas and log them as new rows
// Deploy this as a Web App (Execute as: Me, Who has access: Anyone)
function doGet(e) {
  // Expected parameters: left, right, dleft, dright, optional ts (timestamp)
  var left = e.parameter.left;
  var right = e.parameter.right;
  var dleft = e.parameter.dleft || 0;
  var dright = e.parameter.dright || 0;
  var ts = e.parameter.ts;
  
  if (left === undefined || right === undefined) {
    return ContentService.createTextOutput('Error: missing left or right parameter')
                         .setMimeType(ContentService.MimeType.TEXT);
  }
  
  // Use server time if timestamp not provided
  if (!ts) {
    ts = new Date().toISOString();
  }
  
  var sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();
  
  // Append a new row: [timestamp, left, right, dleft, dright]
  sheet.appendRow([ts, left, right, dleft, dright]);
  
  return ContentService.createTextOutput('Success: appended [' + ts + ', ' + left + ', ' + right + ', ' + dleft + ', ' + dright + ']')
                       .setMimeType(ContentService.MimeType.TEXT);
}
