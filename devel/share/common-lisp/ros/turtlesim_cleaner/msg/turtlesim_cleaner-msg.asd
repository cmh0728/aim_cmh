
(cl:in-package :asdf)

(defsystem "turtlesim_cleaner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MyCustom" :depends-on ("_package_MyCustom"))
    (:file "_package_MyCustom" :depends-on ("_package"))
  ))