/**
 * Function to send a message from the webkit to python backend
 * works by changing the title of the page to the message to be send
 * @param msg message to send to python backend
 */
function send(msg) {
    document.title = "null";
    document.title = msg;
}