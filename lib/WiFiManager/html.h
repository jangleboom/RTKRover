#ifndef HTML_H
#define HTML_H

const char INDEX_HTML[] =
"<!DOCTYPE HTML> <html> <head> <meta content=\"text/html\"; charset=\"UTF-8\"; http-equiv=\"content-type\"> <meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0\"> <title>WiFi setup</title> <style>body { background-color: #CB1211; font-family: Arial, Helvetica, sans-serif; Color: #000000; text-align:center; border: none;} </style> </head> <body> <h2>WIFI SETUP</h2> <h3>Enter new credentials</h3> <form action=\"/\" method=\"post\"> <p> <input maxlength=\"30\" name=\"ssid\" placeholder=\"Personal hotspot SSID\" style=\"text-align:center;\"><br> <input maxlength=\"30\" name=\"password\" placeholder=\"Personal hotspot password\"style=\"text-align:center;\"><br> </p> <p> <input type=\"submit\" value=\"Save\" style=\"height:30px; width:50px; background-color: #000000; Color: #FFFFFF; border: none;\"> </p> </form> </body> </html>";

#endif /* HTML_H */