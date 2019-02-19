# MS1 Messages Package
This package contains the messages that will be used in MS1 Integration Meeting
The list of msgs contained is:

* ActionSeq.msg
* Human.msg
* Humans.msg
* Object.msg
* Objects.msg

--------------------------------------
### Generate A new Message in the Package:

1. Create a ```.msg``` file that contains your message in the *msg* folder. (i.e. example.msg)

2. Open *CMakeLists.txt* and add your .msg file in the following block of code:

```python
add_message_files(
  DIRECTORY msg
  FILES
  StringStamped.msg
  ActionSeq.msg
  Human.msg
  Humans.msg
  Object.msg
  Objects.msg
  )
```
