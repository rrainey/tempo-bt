# Tempo Insights Application

**Tempo Insights** will be a web-based application designed to securely collect and analyze jump logs collected by Tempo-BT jump logging devices. The application will be designed to run on Raspberry Pi 5 servers. It will include a Bluetooth-capable component designed to interact at the local drop zone with logging devices with ontly a minimal requirement for user interactions with the devices.  Under normal operating circumstances, the User/Jumper simply turns the device on, makes a jump, and the Tempo Insights application will automatically discover, upload, and securely store information about the jump.  The Jumper/User can then log into the application via the web and review their results.

## Core Application Objects

**Users** A user provides an e-mail address, their Full Name, and a password, which is stored as a secure hash. Users may be granted an "Administrator" role. The system will be initially provisioned with an "Administrator" user. That user will automatically be granted the Administrator role.  Some application activities will only be available to Administrators - these will be described later.
**Jump Log** A user will accumulate jump logs in the application via one or more Devices assigned to them.  The Jump Log record will consist of the integer Jump Number (which will be assigned automatically), the date/time of the jump (local time zone; this will be an estimated timestamp of the exit from the aircraft, estimated by reviewing the log contents - more details on the data model requirements later), a blob containing the actual jump data file received from the logging device, a SHA-256 hash code of the log file contents, and user editable notes about the jump. A jump log record is always visible to the owning user.  A "Visible to Connections" flag is also associated with the record.  By default, this will be True when the log record is first gathered.  A user can make the record hidden from others, by clearing this "Visible to Connections" flag. A user will have zero or more associated Jump Logs.
**Groups** Groups are collections of Users. A User may be a member of zero or more Groups. A User may be an Administrator or a Member of a Group.  A User may create a Group on demand from the UI.  They specify a name and description.  This User automatically become a member of the Group as an Administrator. A Group may be Private or Public.  
**Joining Groups** All public Groups are listed in a group catalog. Users may join a public group simply by pressing a "Join" button associated with that Group.  To join a private Group, an Administrator of the group must "Invite" a User.  Each User will have a list of zero or more pending Invitations - they may accept the invitation and this become a member of the Group.  They may also "Decline" the invitation, which removes it with no other action taken.
**User Visibility** Users can search for other users by name. A User can "Send Connection Request" to any user appearing in the search. That user will see active Connection Requests in a "Pending Actions" display that groups both Group Membership Invitations and Connection Requests.
**Jump Log Visibility to other users** all their Jump Records marked "Visible to Connections" become reviewable to members of a Group where they are a member.

## Device Management and "Factory Initialization"

**Device ID and Device Name** Each device will be assigned a unique Bluetooth ID. Details of how this will be allocated will be decided later. The Bluetooth name of the device will be "Tempo-BT-xxxx" where "xxxx" is the last four hex digits of the assigned Bluetooth ID.  For device "Factory Initialization", the device is first Flashed with application code and then powered up.
**Device Database** Devices may be in one of three states:
    - **Unprovisioned** - the device has been flashed  but has not been assigned a unique Bluetooth ID yet. It communicates on Bluetooth with the name "Tempo-BT-unprovisioned".
    - **Provisioned** - a Unique ID has been assigned and is stored in NV memory but the device has not been assigned to a User. [on the device, when a provisioning operation is complete, the device will rapidly (2Hz) flash the RGB LED Blue for ten seconds as an extra indication of which device was just provisioned.]
    - **Assigned** - the device has been assigned to a user.  The UUID or the user along with the "nextJumpNumber" are both stored in a uinfo.json file in the root directory of the SD Card.
**Administrator Role and Device Management** An User having the Administrator role, will have a "Device Management" Menu item available. Selecting that will take the user to a "Device Management" page.  The page will list all known devices and all freshly flashed but "Unprovisioned" devices (these will be discovered and recorded by the app's Bluetooth Service, which will be described later). A Device is considered "Provisioned" once it is assigned to a specific user.  A set of menu items (exposed via a "kebob" icon on each Device) on each Device will allow the device to be "Assigned to User", "Initialize", "Unprovision"ed, or "Blink" (Rapid blink the RGB LED amber for ten seconds). Each device on the display will have an "On-line" status, indicating that it has been detected in the last 5 minutes by the Bluetooth Service.  Devices must be on-lin to be assigned to a user or unprovisioned.  When being assigned, the Administrator selects the target User for assignment and also has the opportunity to set a "Next Jump Number". If this value isn't entered, it defaults to 1.


## Log Discovery and Upload Service

### General Description
A Bluetooth-capable process will run continuously on the application server.  It will continuously scan for "Tempo-BT" devices via BLE and maintain a table of all [Tempo devices](https://github.com/rrainey/tempo/tree/main/zephyr/tempo-bt-v1) it detects. Where a named Tempo-BT device shows up that has not been "seen" in over DISCOVERY_WINDOW (configurable, default: 300) seconds, the service will query the device for any log files that it has not successfully uploaded. It can detect this by storing the on-device pathname of the log file with the pathnames present on the device. Any files which have not previously been uploaded should be transferred to the server as new Jump Log records. The User ID metadata from the device should be used to determine which User ine the application database that the Log will be bound to (this get/set User UUID capability remains to be added to the device firemware).  This BLE scanning will be continuous.  After first detection, the device will be marked a "On-line". If it isn't detected by polling for over DISCOVERY_WINDOW seconds, it's status is updated to "Offline".

### Preliminary Jump Analysis and Grouping New Uploads into Formation Jumps

Newly arriving jump log records will be queued for analysis. The queue is determined by querying all Log records that have a NULL "initialAnalysisTimestamp", ordered by creation timestamp (newest first). Each record is analyzed serially at the server by a periodically running worker job.  the worker wakes up every 30 seconds, processes all jumps in the queue, then sleeps for another 30 seconds before reactivating.

#### Initial Analysis

When a new log file is processed by the worker, the data stream will be analyzed to extract or estimate several useful pieces of information:
- time offset for exit (estimated, seconds, relative to start of log) - sustained rate of descent over 2000 fpm for 1 second.
- time offset for parachute activation (estimated,seconds, relative to start of log) - estimated by first significant vertical deceleration (0.25g for 0.1 seconds for rough draft purposes)
- time offset for parachute deployment (seconds, relative to start of log) - first rate of descent below 2000 fpm after being in freefall
- time offset for landing (seconds, relative to start of log) - rate of descent <100 fpm for 10 seconds
- estimated exit timestamp (UTC)
- GNSS position (lat/lon) at estimated exit time.

These data items will be recorded in the Jump Log record.

#### Identifying Formation Jumps

The log start time should be compared with that of all other jump logs.  If the start time of the jump is within 120 seconds of any other jump, those jumps will be logically grouped into a **Formation Skydive**. The log records for any Formation Skydive will be visible to all Users who participated in the jump (and participation is determined by the presence of their own log record for that jump). The Log Records for a Formation Skydive will accumulate as log records for different devices are uploaded.  Eventually all pertinent logs will be uploaded and the Formation Skydive will reflect available data to all participants in the jump.

## Jump Review

### Formation Skydive Review

The (GNSS) position and pressure altitude data for each jumper will be used to render a "movie" of the skydive. A specific jumper will be designated as the "base" skydiver.  All other jumper locations and relative altitude differences will be plotted relative to this base jumper. The +X axis for the "Formation Frame" will be the Forward ground track of the jump aircraft. +Z is Up, and +Y maintains a right-hand coordinate system for the "Formation Frame".  A data set is then generated based on each log, translating the GPS lat/lon information into X/Y positions and Z locations  will be based on pressure altitude difference (cartesian units selectable as feet or meters).  These points can then be plotted using a rendering box that can be caged to a vertical "god's eye" view, caged looking  at the base from -X, or manually rotated to get different vantage points (left mouse down rotates the frame, scroll wheel zooms in and out - always centering on the designated base jumper). Reverse/Pause/Play/Fast-Forward controls will be present and allow for better interactive control in a debrief. A legend will depict the base jumper's fall rate (expressed in mph) and the altitude expressed as ft AGL.

### Fall Rate Performance Assessment
A Jumper's fall rate is governed by many factors simultaneously. All tother factors being equal, heavy jumpers fall faster. A jumper can select different jump suit styles, which will affect drag. And local air density will affect fall rate - and Standard Atmosphere air density will vary based on altitude and is also affected by both temperature and humidity.

The aerodynamic force acting on a body in freefall can be expressed as:

$$ F = 1/2 C_D \rho v^2 $$

Where $ C_D $ is the coefficient of Drag and $ \rho $ is air density.

For non-accelerating motion, the vertical force ($F$) will be equal to the jumper's weight ($W$), so:

$$ W = 1/2 C_D \rho v^2 $$

In many ways, a successful formation skydive will depend on matching the fall rates of all the participants.  Finding matched fall rates is a trial and error process today.  Our goal is to give jumpers solid guidance on how their fall rate range compares with an "average" jumper. A skydiver can control their fall rate relative to other jumps through changes in their body position. "Get big and de-arch" to go slower.  "Get small or arch hard" to go faster.  

We propose designing a "Fall Rate Baseline" Dive Flow to gather data about a jumper's fall rate range: the jumper ride the plane to full altitude, normally around 13,000 feet AGL, and exits solo.  The jumper "gets big" counting to fifteen, then "gets small" and counts to fifteen again. They repeat this alternating sequence down to their deployment altitude.  This should give us a good data set to evaluate their slowest and fastest belly-to-earth fall rates.

The rate of descent can be evaluated from the fall rate of change in altitude (which, in turn, is based on air pressure sampling). For analysis, this fall rate should be adjusted by computing looking up the air density for the given altitude. Air density will change based on the density lapse rate of the Standard Atmosphere - air is thinner at higher altitudes, so - al other factors being equal - a jumper will fall faster at higher altitudes. We propose that a correction can be applied along the descent to "normalize" the fall rate to correct for the local air density to develop a "normalized fall rate" reading which can be plotted.  We can analyze the plot to locate the period of faster and slower falling - and withing each of those intervals take perhaps samples from the stable middle section of each to develop a max/min range for fall rate.  That range could then be visually compared against a predetermined rage for an "average jumper" to give the skydiver feedback on how they compare - and help them select to either alter their jumpsuit configuration or perhaps wear weights when jumping.  The min / max fall rates used in this depiction should be tunable as system system settings, with the defaults being 110 and 130 mph, respectively.

## Lending a Device

Device Lending will introduce the concept of a **Proxy User**.  Where a User lends a device to someone who doesn't yet have a app User account, a Proxy User will be created.  A Proxy User is visible to Users in the same way that a full User account is visible.  Jump log data will be associated with the Proxy User in the same way that it is associated with an Account-holding User - Prox Users and their data will be visible to the User who created the Proxy User.  Given all that, a Proxy User is probably best implemented as a special state or flag associated with a User object.

A User will want to be able to lend a device to another skydiver for one or more jumps.  This will be called **Lending**. A "Lend My Device" link will be exposed on the User's Home page. It links to a page where the User can select the device to be lent (this User may "own" more than one device), specify the User to lend to, and the lending period:

- A User may own more than one device; a dropdown box allows the user to select which device they intend to lend.
- A widget will allow for easy text search and selection of existing users.  This widget will also allow for a "Create Proxy User.." function. The lending User adds the Full Name of the new User,
- The lending period can be "one jump", "any number of jumps", or "until I reclaim the device". This information will be stored in the Device's object record and used to potentially automatically "return"

A person with a Proxy User account created on their behalf may claim their account.  If a User has created a Proxy User, they can "Help Someone Claim their Account". Their list of Proxy Users who they have lend a device to is shown.  They select a User.  The application will generate one-time "Claim Account" URL - this is a URL to "Create Account" Page specifically to change an existing Proxy User account into a regular account along with a parameter specifying the Claim Account record - that, in turn, specifies the UUID of the Proxy Account.  When a Proxy User is selected on this page, a QR Code corresponding to the "Claim Account" url will be displayed. The person who'd like to claim the account can use their phone's camera to capture the URL and claim their account.

## Data Model Notes

Here's notes about the minimal fields we'd expect to have present in each application data object.

### Users
- A UUID to uniquely identify the User (auto generated)
- Full Name
- Profile image (blob, 2MB limit)
- Profile image MIME content type (e.g., "image/jpeg", "image/png")
- An "isProxy" flag
- Where it represents a Proxy User, a reference to the "proxyCreator" user
- creation timestamp UTC
- last active timestamp UTC
- a "User Slug" - used to form the url of the user; must be unique (automatically generated to define a URL for the group; a lowercase, whitespace-mapped version of the friendly name; e.g. "Bill Jones" should be mapped to "bill-jones"; when a user is created and a "slug" already exists, the generated slog will have a "-nnn" appended, where nnn is a randomly generated decimal number)
- Assigned User Roles

### User Roles
Currently the only defined user role is (system) "Administrator"
- UUID of User (indexed) 
- Role enumeration **currently** only Administrator is defined.

Together, these two elements form a composite primary key for the table.

### Groups
- A UUID (auto generated)
- Friendly name
- a "Slug" (automatically generated to help define a URL for the group; a lowercase, whitespace-mapped version of the friendly name; e.g. "Camp Casual Fridays" should be mapped to "camp-casual-fridays"; when a group is created and the mapped "slug" already exists, the generated slug will have a "-nnn" appended, where nnn is a randomly generated decimal number)
- Map of Users (with isAdmin flag)

### Jump Log
= a UUID (auto generated)
- Log Start Timestamp (UTC)
- Estimated Exit Lat/Lon (possibly null)
- estimated exit altitude (feet, possibly null)
- owning User UUID
- Jump Number for the user
- Log contents (long text or blob, whatever is most efficient for 8MB log files)
- time offset for exit (estimated, seconds, relative to start of log) - sustained rate of descent over 2000 fpm for 1 second.
- time offset for parachute activation (estimated,seconds, relative to start of log) - estimated by first significant vertical deceleration (0.25g for 0.1 seconds for rough draft purposes)
- time offset for parachute deployment (seconds, relative to start of log) - first rate of descent below 2000 fpm after being in freefall
- time offset for landing (seconds, relative to start of log) - rate of descent <100 fpm for 10 seconds
- User editable markdown text notes describing the jump
- the SHA-259 hash of the Jump log contents
- a "visibleToConnections" Flag - defaults to True when a log is uploaded, but the owning user can edit this value.
- app record creation timestap (UTC) - timestamp of record creation
- initialAnalysisTimestamp - the date/timestamp of completion of initial analysis.
- initialAnalysisMessage - text; any unusual conditions or errors encountered during analysis are logged here.


### Formation Skydive
- a UUID (auto generated)
- A Map of a Jump Logs associated with this Formation skydive (within each map entry is a "isVisibleToOthers" flag; this flag defaults to True, but can be reset by the owning User to make the Jump data invisible to others).
- UUID of the Jump log used as the "Base" jumper (editable by anyone participating in the Skydive)
- date of the jump (local time zone) - derived from the start time from the first two Jumps that comprised the skydive.

The data represented by this Formation Skydive and the log data for all jumpers is visible to all participants in the Jump (after taking int account the "isVisibleToOthers" flag previously mentioned)

### Devices
- An app assigned UUID (for internal tracking purposes) (auto generated)
- Owner User (possibly null)
- Lent-To User (possibly null)
- On-line status
- Device State: Unprovisioned, Provisioned, Assigned
- Device UID *(possibly null)
- Device Bluetooth Name
- Device Bluetooth ID
- Lending Status: Lent-till-reclaimed, one-jump-only

### User Invitation
- UUID of User invitation object (auto generated)
- Expiration timestamp, UTC - generated invitations expire 30 days after being generated
- Proxy User UUID - the id of the user being invited to join.

User Invitations are only meaningful when a User has lent their device to someone for a jump and wants to allow that user to register and gain direct access to the data.  Otherwise, the new user can just register by visiting the login page.

## Page mapping

- `/` just a redirect to the login page or /home if logged in, for now.

- `/login`

- `/register`

- `/accept-invitation/[invitation-id]`

- `/profile` - editable user and account settings. Profile Photo. Change password.

- `/home` - a user specific home page. A left-side navigation bar (Groups - the list of Groups the user is a member of, Devices, including status of "loaner" devices, Settings, Logout). Jump review rendered in the main (center) window. Right side blocks include "Formation Jumps", "My Jumps", "Summary" (Summary on the selected jump; date/time (local), exit altitude) - for "solo" jumps, jump number, estimated deployment altitude, freefall time (sec), average fall rate (mph)

- `users/[user-slug]`

- `groups/[group-slug]`

- `review/fs/[fs-id]` - a dedicated review page depicting information for the specified formation skydive. Navigation minimized and information display simplified to essentials. Dots for each jumper with a succinct data block with Name, distance from the base (feet), closure rate to the base (fps & mph). Right hand blocks for Jumper list, and Base Info (fall rate; calibrated fall rate)

## Technology Stack
Use Next.js, Typescript, Mantine for core UI elements, recharts for charts and graphs, d3 for 3D visualization, Prisma for ORM, and a local instance of Supabase for database storage. Avoid using Tailwind CSS for styling, but instead use Mantine's organic styling extensibility.

The application should be designed for deployment on Ubuntu 24.04 on a Raspberry Pi 5.

The application will be deployed at the local dropzone facility to provide for Bluetooth connectivity to the logging devices.  The Bluetooth functions should be provided through the most reliable Bluetooth stack available that can provide all needed interaction functions - I'd prefer to use BluZ directly from Typescript, but using mcumgr via shell callouts might be an option as well.

## Theming

The application should use these colors, aiming for a overall Dark Mode color scheme.

color-1 #002233 (primary background)
color-2 #11425d (secondary backgrounds)
color 3 #ddff55 (emphasis font/line color)
color-4 #c0d6ea
color-6 #c5c0c9 (text foreground, primary line drawing/graphics)
color-7 #f6f2e8
color-8 #f04848 (flag unsafe values or conditions on the graphical depictions)
color-9 #855bf0 (alternate emphasis font/line color)

Use the Mantine default per-platform font choices.