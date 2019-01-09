
function switchTest(){
    var readlineSync =require ('readline-sync')
    var f;
    var index;
    var functions;
functions = ['getMission', 'getObstacles', 'postOdlcs','getOdlcs', 'putOdlcs','telemetry']
index = readlineSync.keyInSelect(functions, 'Which function do you wanna test? ');
f= functions[index];

console.log('Ok, ' + functions[index] +' is being tested.');

switch(f)
{
    case 'getMission':
    getMissions(cookieobj.cookie);
    break;
    case 'getObstacles':
    getObstacles(cookieobj.cookie);
    break;
    case 'postodlcs':
    postOdlcs(cookieobj.cookie, odlcs_1);
    break;
    case 'telemetry':
    telemetry(cookieobj.cookie,Data)
    break;
    case'getOdlcs':
    getOdlcs(cookieobj.cookie);
    break;
    case 'putOdlcs':
    putOdlcs(cookieobj.cookie,alphabet)
    default:
    console.log('went to default pl check');
}

}
setTimeout(() => console.log(cookieobj.cookie), 2000);
setTimeout(() => (getMissions(cookieobj.cookie)), 2001);
setTimeout(() => (getObstacles(cookieobj.cookie)), 2002);
setTimeout(() => (postOdlcs(cookieobj.cookie, odlcs_1)), 2003);
setTimeout(() => (telemetry(cookieobj.cookie,Data)), 2004);
setTimeout(()=> (getOdlcs(cookieobj.cookie)),2005);
setTimeout(()=> (putOdlcs(cookieobj.cookie,alphabet)),2006);
