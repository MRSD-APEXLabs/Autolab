# Getting Started with Create React App

This project was bootstrapped with [Create React App](https://github.com/facebook/create-react-app).

## Testing

### Unit tests (frontend)

```bash
npm install    # first time, or after package.json changes
npm test       # interactive watch mode
CI=true npx react-scripts test --watchAll=false            # run once, non-interactive (CI)
CI=true npx react-scripts test src/debug --watchAll=false   # just the debug console suite
```

### Debug / Testing Console (manual)

The `/debug` route (`src/debug/`) is a curated low-level command console for testing
individual autonomy subsystems (navigation, planning, perception, manipulation, lab
machine integration) outside the main APEX flow. See
`docs/superpowers/specs/2026-09-13-debug-testing-ui-design.md` for the full design.

To exercise it end to end:

1. Bring up the stack so the MQTT broker and ROS2 nodes are running:
   ```bash
   autolab up            # from the repo root, sitl/desktop profile is fine
   ```
2. Start the dev server and open the console:
   ```bash
   npm start
   # then visit http://localhost:3000/debug
   ```
3. The connection banner at the top should read "MQTT: connected" once the
   broker (port 9001, MQTT-over-WebSocket) is reachable. Each panel's status
   tiles populate as the corresponding ROS2 topics publish; every
   motion/hardware-sending button requires a second "Confirm?" click before
   it actually publishes.

### ROS2-side tests (bridge + manipulation_executive)

These live outside `gcs/ui` and need a container with ROS2 sourced (see the
top-level `CLAUDE.md`):

```bash
# Inside the gcs container — mqtt_ros2_bridge unit tests
colcon test --packages-select gcs_monitoring --event-handlers=console_direct+

# Inside the robot container — manipulation_executive's camera-mode guard test
colcon build --symlink-install --packages-select manipulation_executive
colcon test --packages-select manipulation_executive --event-handlers=console_direct+
```

## Available Scripts

In the project directory, you can run:

### `npm start`

Runs the app in the development mode.\
Open [http://localhost:3000](http://localhost:3000) to view it in your browser.

The page will reload when you make changes.\
You may also see any lint errors in the console.

### `npm test`

Launches the test runner in the interactive watch mode.\
See the section about [running tests](https://facebook.github.io/create-react-app/docs/running-tests) for more information.

### `npm run build`

Builds the app for production to the `build` folder.\
It correctly bundles React in production mode and optimizes the build for the best performance.

The build is minified and the filenames include the hashes.\
Your app is ready to be deployed!

See the section about [deployment](https://facebook.github.io/create-react-app/docs/deployment) for more information.

### `npm run eject`

**Note: this is a one-way operation. Once you `eject`, you can't go back!**

If you aren't satisfied with the build tool and configuration choices, you can `eject` at any time. This command will remove the single build dependency from your project.

Instead, it will copy all the configuration files and the transitive dependencies (webpack, Babel, ESLint, etc) right into your project so you have full control over them. All of the commands except `eject` will still work, but they will point to the copied scripts so you can tweak them. At this point you're on your own.

You don't have to ever use `eject`. The curated feature set is suitable for small and middle deployments, and you shouldn't feel obligated to use this feature. However we understand that this tool wouldn't be useful if you couldn't customize it when you are ready for it.

## Learn More

You can learn more in the [Create React App documentation](https://facebook.github.io/create-react-app/docs/getting-started).

To learn React, check out the [React documentation](https://reactjs.org/).

### Code Splitting

This section has moved here: [https://facebook.github.io/create-react-app/docs/code-splitting](https://facebook.github.io/create-react-app/docs/code-splitting)

### Analyzing the Bundle Size

This section has moved here: [https://facebook.github.io/create-react-app/docs/analyzing-the-bundle-size](https://facebook.github.io/create-react-app/docs/analyzing-the-bundle-size)

### Making a Progressive Web App

This section has moved here: [https://facebook.github.io/create-react-app/docs/making-a-progressive-web-app](https://facebook.github.io/create-react-app/docs/making-a-progressive-web-app)

### Advanced Configuration

This section has moved here: [https://facebook.github.io/create-react-app/docs/advanced-configuration](https://facebook.github.io/create-react-app/docs/advanced-configuration)

### Deployment

This section has moved here: [https://facebook.github.io/create-react-app/docs/deployment](https://facebook.github.io/create-react-app/docs/deployment)

### `npm run build` fails to minify

This section has moved here: [https://facebook.github.io/create-react-app/docs/troubleshooting#npm-run-build-fails-to-minify](https://facebook.github.io/create-react-app/docs/troubleshooting#npm-run-build-fails-to-minify)
