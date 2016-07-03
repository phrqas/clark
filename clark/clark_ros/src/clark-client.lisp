;;;; Copyright (c) 2016 Massachusetts Institute of Technology
;;;;
;;;; This software may not be redistributed, and can only be retained and used
;;;; with the explicit written consent of the author, subject to the following
;;;; conditions:
;;;;
;;;; The above copyright notice and this permission notice shall be included in
;;;; all copies or substantial portions of the Software.
;;;;
;;;; This software may only be used for non-commercial, non-profit, research
;;;; activities.
;;;;
;;;; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESSED
;;;; OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
;;;; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
;;;; THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
;;;; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
;;;; FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
;;;; DEALINGS IN THE SOFTWARE.
;;;;
;;;; Authors:
;;;;   Pedro Santana
(in-package #:roslisp)

(defclass clark-cl-client ()
  ((node-name
    :initarg :node-name
    :initform "clark-cl-client"
    :documentation "Name of the ROS node associated with this CLARK client.")
    (clark-service-name
     :initarg :clark-service-name
     :initform "clark_service"
     :documentation "Name of the CLARK ROS service.")
    (clark-service-timeout
     :initarg :clark-service-timeout
     :initform 10
     :documentation "Number of seconds that the client waits to connect to the service before returning an error.")
    (node-status
     :documentation "Stores the ROS node initialization status.")
   ))

(defmethod initialize-instance :after ((client clark-cl-client) &key)
  (with-slots ((nn node-name) (ns node-status) (cs clark-service-name) (to clark-service-timeout)) client
    (format t "~%Creating CLARK client node...")
    (setf ns (start-ros-node nn))
    (format t "~%Waiting to connect to the CLARK service...")
    (if (wait-for-service cs to)
      (format t "connected!")
      (error "Failed to connect to CLARK service"))))

(defmethod call-clark ((client clark-cl-client) &key type params)
  "Calls the CLARK ROS service."
  (let (resp)
    (with-slots ((cs clark-service-name)) client
       (setf resp (call-service "clark_service" 'clark_ros-srv:CLARKSrv
                     :type type
                     :params params)))
    resp))

(defmethod shutdown-client ((client clark-cl-client))
    (shutdown-ros-node))
