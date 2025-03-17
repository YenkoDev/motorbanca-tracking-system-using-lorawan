function Decoder(bytes, port) {
  var char_results = "";
  for (var i = 0; i < bytes.length; i++) {
    char_results += (String.fromCharCode(bytes[i]));
  }
  const parts = char_results.split("&");
  
  var decoded = {
    field1: parseFloat(parts[0]),
    field2: parseFloat(parts[1]),
    field3: parseFloat(parts[2]),
    field4: parseFloat(parts[3]),
    field5: parseFloat(parts[4]),
    field6: parseFloat(parts[5]),
  };
  
  return decoded;
}
