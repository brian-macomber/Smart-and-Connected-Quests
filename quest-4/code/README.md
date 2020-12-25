# Code Readme

### Brian: fob_code.c

The fob_code.c file is the same code file for each ESP in the system, except for the ID number that is unique to each fob. The FOB will have the leader election elgorithm run, which is an implementation of the Bully Algorithm. Each ESP will start out with no leader, and will ping ESPs around it looking for a leader. If it times out after 30 seconds, it will elect itself as the leader. In order for the ESPs that are followers to know that the leader is alive, the leader will send out a heartbeat to each ESP in the system every 2 seconds.

No matter the state of the esp, button 1 on the fob will cycle between the 3 different candidates (red, blue, green) , and the second button will send the data over IR. In addition to this, when the current esp is the leader, the onboard LED will be lit to show this.

### Ritam: server.js

The server.js file was responsible for the following: creating the webclient, sending and receiving messages from the webclient using socket.io, creating the database, and adding entries (votes) to the database once they were received from the leader ESP. The server also emitted a message to the frontend containing the desired queries.

### Ram: index.html

The frontend aspect of this Quest revolved around getting the proper queries from the server to be displayed on the client. We set up the format of the html file to have two tables (one for candidate and total votes, one for the entire database), and a reset button.
We used socket.io with our levelDB queries in order to retrieve the data properly. The format of the message being sent from the server would be given in a specific array which we would have to properly index into. We created different variables to hold different values from the respective array index. This allowed us to see when a message was being sent over- or in this case, when a vote was being cast.
The biggest problems we faced was displaying the javascript variables from inside the socket.io portions and inside functions. HTML had a couple of ways to do it such as "Document.write" and through .innerHTML. Displaying these values in our tables was a problem since it would not register the actual values. Usually, we got an error such as undefined value or a blank segment. We solved this issue by changing the order of the .innerHTML and other functions. We were able to get an output of the vote count for each candidate, and it was able to register when a vote was cast. However, the counter for the function we created was not properly incrementing. This was due to the code just sending signal messages that indicate when to increment, but we did not have a way to reset the vote counter right before the socket.io was notified of the message. If we tried resetting the values of vote counts inside the socket.io, it would just output 0. We were able to successfully get the full query for fobID, and the other information, but we were not able to display it as part of a full table. The values would not register or show up; a lot of the issues we faced came from accessing javascript variables and communication from socket.io and the backend. Unfortunately, the reset button had a flag which should send a io.emit back to the backend to clear the database, but that event was not getting properly triggered or identified.
